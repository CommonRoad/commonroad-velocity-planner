import copy
from dataclasses import dataclass
import logging


import numpy as np
import scipy
from scipy.interpolate import CubicSpline
from scipy.signal import argrelextrema
from shapely import LineString, intersection_all, Point


# Own code base
from commonroad_velocity_planner.planner.base_planner import BaseVelocityPlanner
from commonroad_velocity_planner.configuration.velocity_planner_config import (
    VelocityPlannerConfig,
)
from commonroad_velocity_planner.velocity_planning_problem import (
    VelocityPlanningProblem,
)
from commonroad_velocity_planner.spline_profile import SplineProfile
from commonroad_velocity_planner.utils.max_velocity_calc import (
    calculate_lat_acc_limited_velocity,
)

# typing
from typing import Optional, Tuple, Callable, List, Union


@dataclass
class VehicleState:
    velocity: Optional[float]
    v_x: Optional[float]
    v_y: Optional[float]
    acceleration_x: Optional[float]
    acceleration_y: Optional[float]
    acceleration: Optional[float]
    arclength: Optional[float]
    f_nominal: Optional[float]
    f_tangential: Optional[float]
    orientation: Optional[float]

    def sanity_check(self) -> bool:
        retval = True
        if self.velocity is not None:
            if self.velocity <= 0:
                _logger = logging.getLogger(
                    name="IVelocityPlanner.planner.bang_bang_planner.VehicleState"
                )
                _logger.warning("velocity is smaller than zero")
                retval = False

        return retval


def factory_vehicle_state(
    velocity: Union[float, None] = None,
    v_x: Union[float, None] = None,
    v_y: Union[float, None] = None,
    acceleration: Union[float, None] = None,
    arclength: Union[float, None] = None,
) -> VehicleState:
    """
    Factory method to start initialization of Vehicle state
    :param velocity:
    :return: partially initialized vehicle state object
    """
    return VehicleState(
        velocity=velocity,
        v_x=v_x,
        v_y=v_y,
        acceleration=acceleration,
        acceleration_x=None,
        acceleration_y=None,
        arclength=arclength,
        f_nominal=None,
        f_tangential=None,
        orientation=None,
    )


def integrate_vehicle_state(
    vehicle_state: VehicleState, dt: float, minimum_velocity: float = 2.5
) -> VehicleState:
    """
    Integrate vehicle state with _dt
    :param vehicle_state:
    :param dt:
    :return:
    """
    s = vehicle_state.velocity * dt + vehicle_state.arclength
    v = max(
        minimum_velocity, vehicle_state.acceleration * abs(dt) + vehicle_state.velocity
    )

    return VehicleState(
        velocity=v,
        v_x=v,
        v_y=0,
        acceleration=None,
        acceleration_x=None,
        acceleration_y=None,
        arclength=s,
        f_nominal=None,
        f_tangential=None,
        orientation=None,
    )


class BangBangSTPlanner(BaseVelocityPlanner):
    def __init__(self, config: VelocityPlannerConfig, dt: float = 0.1) -> None:

        super().__init__(velocity_planner_config=config)

        self._logger = logging.getLogger(
            name="IVelocityPlanner.planner.bang_bang_planner"
        )

        self._config: VelocityPlannerConfig = config
        self._f_f_max = self._calculate_max_front_force()
        self._dt = dt

    @property
    def dt(self) -> float:
        """
        :return: delta_t for computation
        """
        return self._dt

    @property
    def max_front_tire_force(self) -> float:
        """
        :return: maximum front tire force
        """
        return self._f_f_max

    def plan_velocity(
        self,
        problem: VelocityPlanningProblem,
        config: Optional[VelocityPlannerConfig] = None,
    ) -> SplineProfile:
        """
        Plan velocity profile
        :param problem: velocity planning problem
        :param config: planner config
        :return: spline profile
        """
        # update config
        self._config = config if config is not None else self._config

        # get relative local maxima
        maxima = argrelextrema(np.abs(problem.path_curvature), np.greater)

        # arc length, curvature and derivative
        curvature = CubicSpline(problem.path_length_per_point, problem.path_curvature)
        curvature_derivative = curvature.derivative()

        # get interpolated velocity around maxima
        velocities: List[CubicSpline] = list()
        for idx_maxima in maxima[0]:
            v_spline: CubicSpline = self._calculate_velocity_around_maximum(
                curvature=curvature,
                curvature_derivative=curvature_derivative,
                s_start=problem.path_length_per_point[idx_maxima],
                s_end=problem.path_length_per_point[-1],
            )
            velocities.append(v_spline)

        # Add initial state as pseudo maximum
        states_from_start: List[VehicleState] = self._integrate_zero_slip(
            curvature=curvature,
            curvature_derivative=curvature_derivative,
            s_start=problem.path_length_per_point[problem.sampled_start_idx],
            s_end=problem.path_length_per_point[-1],
            v0=problem.v_initial,
        )
        s_from_start: np.ndarray = np.asarray(
            [state.arclength for state in states_from_start]
        )
        v_from_start: np.ndarray = np.asarray(
            [state.velocity for state in states_from_start]
        )
        velocities.append(CubicSpline(s_from_start, v_from_start, extrapolate=False))

        # Add final state as pseudo maximum
        states_from_end: List[VehicleState] = self._integrate_zero_slip(
            curvature=curvature,
            curvature_derivative=curvature_derivative,
            s_start=problem.path_length_per_point[problem.sampled_goal_idx],
            s_end=problem.path_length_per_point[problem.sampled_start_idx],
            v0=problem.v_stop,
        )
        s_from_end: np.ndarray = np.asarray(
            [state.arclength for state in reversed(states_from_end)]
        )
        v_from_end: np.ndarray = np.asarray(
            [state.velocity for state in reversed(states_from_end)]
        )
        velocities.append(CubicSpline(s_from_end, v_from_end))

        # at each arc length calculate velocity profile
        vel_prof: List[float] = list()
        for s in problem.path_length_per_point:
            # 1. only positive values
            positive_values: List[float] = [
                max(self._config.v_min_driving, interpolated_spline(s))
                for interpolated_spline in velocities
            ]
            vel_prof.append(min(positive_values))

        vel_prof: np.ndarray = np.array(vel_prof)

        # 4. cut with velocity profile with speed limits and lateral acceleration
        lateral_acceleration: np.ndarray = calculate_lat_acc_limited_velocity(
            problem=problem, config=self._config, v_max=self._config.v_max_street
        )
        vel_prof = np.minimum(vel_prof, problem.speed_limits)
        vel_prof = np.minimum(vel_prof, lateral_acceleration)

        return SplineProfile(
            path_length_per_point=problem.path_length_per_point,
            velocity_profile=vel_prof,
        )

    def _calculate_max_front_force(self) -> float:
        """
        Calculates maximum front tire force
        :return: maximum front tire force
        """

        def total_friction_force_front(x):
            return -self._config.vehicle_config.tire_D_front * np.sin(
                self._config.vehicle_config.tire_C_front
                * np.arctan(self._config.vehicle_config.tire_B_front * x)
            )

        res = scipy.optimize.minimize_scalar(total_friction_force_front, bounds=(0, 1))
        F_fz = (
            self._config.vehicle_config.mass
            * self._config.g
            * self._config.vehicle_config.length_rear
            / (
                self._config.vehicle_config.length_front
                + self._config.vehicle_config.length_rear
            )
        )
        return -F_fz * res.fun

    def _calculate_initial_velocity(self, curvature: float, n=200) -> float:
        """
        Calculates initial velocity
        :param curvature: curvature at initial state
        :param n:
        :return: initial velocity
        """
        curvature = np.abs(curvature)

        # FIXME: problems with negative curvature?
        if curvature < 1.0e-5:
            curvature = 1.0e-5

        s_ry = -self._config.vehicle_config.length_rear * curvature
        f_rx, f_ry = self._calc_rear_friction_circle(s_ry, n)
        r_friction_circle = LineString(np.array([f_rx, f_ry]).T)
        gg_diagram = r_friction_circle.buffer(self._f_f_max)

        f_dir = LineString([[0, 0], [0, 100000]])
        f_n = intersection_all([gg_diagram, f_dir]).length
        v = np.sqrt(f_n / curvature / self._config.vehicle_config.mass)
        return v

    def _calculate_velocity_around_maximum(
        self,
        curvature: Callable[[float], float],
        curvature_derivative: Callable[[float], float],
        s_start: float,
        s_end: float,
    ) -> CubicSpline:
        """
        Cubic spline interpolated velocity around local maximum
        :param curvature: current curvature
        :param curvature_derivative: curvature derivative
        :param s_start: start arc length
        :param s_end: end arc length
        :return: cubic spline interpolated velocity in region around local maximum
        """
        vehicle_states_forward: List[VehicleState] = self._integrate_zero_slip(
            curvature=curvature,
            curvature_derivative=curvature_derivative,
            s_start=s_start,
            s_end=s_end,
        )
        vehicle_states_backward: List[VehicleState] = self._integrate_zero_slip(
            curvature=curvature,
            curvature_derivative=curvature_derivative,
            s_start=s_start,
            s_end=0,
        )

        # give velocity profile as cubic spline
        arclength_backward = np.asarray(
            [vs.arclength for vs in reversed(vehicle_states_backward)]
        )
        arc_length_forward = np.asarray([vs.arclength for vs in vehicle_states_forward])
        velocity_backward = np.asarray(
            [vs.velocity for vs in reversed(vehicle_states_backward)]
        )
        velocity_forward = np.asarray([vs.velocity for vs in vehicle_states_forward])
        s = np.concatenate((arclength_backward, arc_length_forward[1:]), axis=0)
        v = np.concatenate((velocity_backward, velocity_forward[1:]), axis=0)

        return CubicSpline(s, v, extrapolate=False)

    def _integrate_zero_slip(
        self,
        curvature: Callable[[float], float],
        curvature_derivative: Callable[[float], float],
        s_start: float,
        s_end: float,
        v0: float = None,
    ) -> List[VehicleState]:
        """
        Integrate zero slip st model.
        :param curvature: current curvature
        :param curvature_derivative: curvature derivative
        :param s_start: start arc length
        :param s_end: end arc length
        :param v0: initial velocity
        :return:
        """

        # set initial state velocity
        v_init = max(
            1.0e-3,
            (
                v0
                if v0 is not None
                else self._calculate_initial_velocity(curvature(s_start))
            ),
        )

        # decide input
        if s_start > s_end:
            u = -1
            dt = -self._dt
        else:
            u = 1
            dt = self._dt

        # caculate current state and integrate
        current_state: VehicleState = factory_vehicle_state(
            v_x=v_init, velocity=v_init, v_y=0, arclength=s_start
        )
        solution: List[VehicleState] = [copy.copy(current_state)]
        termination_counter: int = 0
        while (
            u == -1
            and current_state.arclength > 0
            or u == 1
            and current_state.arclength < s_end
        ):
            current_state = self._st_system_zero_slip(
                state=current_state,
                curvature=curvature,
                curvature_derivative=curvature_derivative,
                u=u,
            )
            current_state = integrate_vehicle_state(
                vehicle_state=current_state,
                dt=dt,
                minimum_velocity=self._config.v_min_driving,
            )
            solution.append(copy.copy(current_state))

            termination_counter += 1
            if termination_counter > 10e5:
                raise Exception("Too many iterations in integration")

        return solution

    def _st_system_zero_slip(
        self,
        state: VehicleState,
        curvature: Callable[[float], float],
        curvature_derivative: Callable[[float], float],
        u: int,
    ) -> VehicleState:
        """
        Calculates single-track zero slip system
        :param state: current state
        :param curvature: current curvature
        :param curvature_derivative:
        :param u: bang-bang normalized input. can be -1 or 1
        :return: partially initialized vehicle state
        """
        # vx, vy, s = state

        orientation: float = np.arctan2(state.v_y, state.v_x)
        v: float = np.sqrt(state.v_x**2 + state.v_y**2)

        f_n, f_t = self._calc_zero_slip_forces(
            curvature=curvature(state.arclength),
            curvature_derivative=curvature_derivative(state.arclength),
            v=v,
            u=u,
        )

        a_x: float = (
            f_t * np.cos(orientation) - f_n * np.sin(orientation)
        ) / self._config.vehicle_config.mass
        a_y: float = (
            f_t * np.sin(orientation) + f_n * np.cos(orientation)
        ) / self._config.vehicle_config.mass
        acceleration: float = np.sqrt(a_x**2 + a_y**2)

        state.orientation = orientation
        state.velocity = v
        state.f_nominal = f_n
        state.f_tangential = f_t
        state.acceleration_x = a_x
        state.acceleration_y = a_y
        state.acceleration = acceleration

        return state

    def _calc_zero_slip_forces(
        self,
        curvature: float,
        curvature_derivative: float,
        v: float,
        u: int,
        n: int = 200,
    ) -> Tuple[float, float]:
        """
        Calculate zero slip forces
        :param curvature: current curvature
        :param curvature_derivative:
        :param v: current velocity
        :param u: normalized bang-bang input, can either be -1 or 1
        :param n: number of samples
        :return: tuple[tangential force, nominal force] relative to current path
        """
        curvature = abs(curvature)
        if abs(curvature) < 1.0e-5:
            curvature = 1.0e-5

        s_ry = -curvature * self._config.vehicle_config.length_rear
        f_rx, f_ry = self._calc_rear_friction_circle(s_ry, n)

        f_ry_min = np.min(f_ry)
        f_ry_max = np.max(f_ry)

        def calc_f_t(f_ry):
            return (
                self._config.vehicle_config.mass
                * v**2
                * (
                    self._config.vehicle_config.length_front
                    * curvature
                    * self._config.vehicle_config.mass
                    - self._config.vehicle_config.inertia_z * curvature_derivative
                )
                / (self._config.vehicle_config.inertia_z * curvature)
                - self._config.vehicle_config.mass
                * (
                    self._config.vehicle_config.length_front
                    + self._config.vehicle_config.length_rear
                )
                / (self._config.vehicle_config.inertia_z * curvature)
                * f_ry
            )

        f_t = calc_f_t(f_ry)
        f_fx = f_t - f_rx
        f_fx_curve = LineString(np.array([f_ry, f_fx]).T)

        # bang bang maximum positive input
        if u == 1:
            f_f_max_line_pos = LineString(
                [[f_ry_min - 100, self._f_f_max], [f_ry_max + 100, self._f_f_max]]
            )
            res = intersection_all([f_f_max_line_pos, f_fx_curve])
            if isinstance(res, Point) or res.is_empty:
                f_t = self._f_f_max
            else:
                f_ry = min(res.geoms[0].x, res.geoms[1].x)
                f_t = calc_f_t(f_ry)
                f_t = max(0, f_t)
        # Bang bang maximum negative input
        else:
            f_f_max_line_neg = LineString(
                [[f_ry_min - 100, -self._f_f_max], [f_ry_max + 100, -self._f_f_max]]
            )
            res = intersection_all([f_f_max_line_neg, f_fx_curve])
            if res.is_empty:
                f_t = self._f_f_max
            else:
                if isinstance(res, Point):
                    f_ry = res.x
                else:
                    f_ry = max(res.geoms[0].x, res.geoms[1].x)
                f_t = calc_f_t(f_ry)
                f_t = min(0, f_t)

        f_n = curvature * self._config.vehicle_config.mass * v**2

        return f_n, f_t

    def _calc_rear_friction_circle(
        self,
        s_ry: float,
        n: int = 200,
    ) -> Tuple[float, float]:
        """
        Calculates rear friction circle via pajeckas formula
        :param s_ry: rear lateral slip
        :param n: number of samples for circle
        :return: tuple[longitudinal rear force, lateral rear force]
        """
        F_rz = (
            self._config.vehicle_config.mass
            * self._config.g
            * self._config.vehicle_config.length_front
            / (
                self._config.vehicle_config.length_front
                + self._config.vehicle_config.length_rear
            )
        )
        s_rx = np.linspace(-1, 1, n)
        s_r = np.sqrt(s_rx**2 + s_ry**2)
        f_r = (
            F_rz
            * self._config.vehicle_config.tire_D_rear
            * np.sin(
                self._config.vehicle_config.tire_C_rear
                * np.arctan(self._config.vehicle_config.tire_B_rear)
            )
        )
        f_ry = -s_ry * f_r / s_r
        f_rx = -s_rx * f_r / s_r
        return f_rx, f_ry
