from dataclasses import dataclass
import logging

import numpy as np
from scipy.interpolate import CubicSpline

# commonroad
from commonroad_route_planner.route import Route
from commonroad_route_planner.utility.polyline_operations.polyline_operations import (
    compute_scalar_curvature_from_polyline,
)

# own code base
from commonroad_velocity_planner.behavior.speed_limits import (
    get_speed_limits_from_lanelet_network,
)
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_velocity_planner.utils.planning_problem import (
    get_goal_velocity,
    get_goal_acceleration,
)
from commonroad_velocity_planner.preprocessing.curvature_smoother import (
    SmoothingStrategy,
    ICurvatureSmoother,
)
from commonroad_velocity_planner.utils.planning_problem import (
    project_goal_on_ref_path,
    project_initial_state_on_ref_path,
)
from commonroad_velocity_planner.utils.polyline_operations import (
    cubic_spline_arc_interpolation_2D,
)


@dataclass
class VelocityPlanningProblem:
    """
    Transforms planning problem in velocity planning problem
    """

    planning_problem: PlanningProblem

    sampled_ref_path: np.ndarray

    sampled_start_idx: int
    sampled_goal_idx: int

    interpoint_distance: float
    path_length_per_point: np.ndarray
    path_curvature: np.ndarray
    speed_limits: np.ndarray

    v_initial: float
    v_stop: float
    a_initial: float
    a_stop: float

    def __post_init__(self):
        _ = self.sanity_check()

    def debug_print(self):
        _logger = logging.getLogger(
            name="IVelocityPlanner.velocity_planning_problem.VelocityPlanningProblem"
        )
        _logger.info(f"VPP: a={self.a_initial}")
        _logger.info(f"VPP: v={self.v_initial}")

    def sanity_check(self) -> bool:
        """
        Sanity check for velocity problem dimensions
        :return: true, if correct, else false
        """

        if not (
            self.sampled_ref_path.shape[0]
            == self.path_length_per_point.shape[0]
            == self.path_curvature.shape[0]
            == self.speed_limits.shape[0]
        ):
            _logger = logging.getLogger(
                name="IVelocityPlanner.velocity_planning_problem.VelocityPlanningProblem"
            )
            _logger.warning(
                "\n \n sanity check for velcoty planning problem failed! \n\n"
                f"len sampled refpath: {self.sampled_ref_path.shape[0]} \n"
                f"len sampled path length per proint: {self.path_length_per_point.shape[0]} \n"
                f"len sampled path curvature per proint: {self.path_curvature.shape[0]} \n"
                f"len sampled speed limits: {self.speed_limits.shape[0]} \n"
            )
            retval = False
        else:
            retval = True

        return retval


class VppBuilder:
    """
    Builds the velocity planning problem, smoothes and downsamples reference path.
    """

    @staticmethod
    def build_vpp(
        route: Route,
        planning_problem: PlanningProblem,
        resampling_distance: float = 2,
        smoothing_strategy: SmoothingStrategy = SmoothingStrategy.ELASTIC_BAND,
        default_goal_velocity: float = 0.0,
        default_goal_acceleration: float = 0.0,
        offset_start_idx: int = 2,
        offset_end_idx: int = 2,
    ) -> VelocityPlanningProblem:
        """
        Build velocity planning problem by downsampling and smoothing the reference path first
        :param route: cr route object
        :param planning_problem: cr planning problem
        :param resampling_distance: resampling distance
        :param planning_horizon: planning horizon
        :param smoothing_strategy: smoothing strategy
        :param default_goal_velocity: default goal velocity if problem does not contain one
        :param default_goal_acceleration: default goal velocity, if problem does not contain one.
        :return: velocity planning problem object
        """
        if resampling_distance is None:
            interpoint_distance = route.average_interpoint_distance
        else:
            interpoint_distance = resampling_distance

        start_index = np.argmin(
            np.linalg.norm(route.reference_path - route.initial_state.position, axis=1)
        )

        # get start and stop index of initial reference path
        start_distance = route.path_length_per_point[start_index]
        stop_distance = route.length_reference_path

        # resample before smoothing
        resampled_reference_path, path_length_per_point = (
            cubic_spline_arc_interpolation_2D(
                polyline=route.reference_path,
                start_distance=start_distance,
                stop_distance=stop_distance,
                resampling_distance=resampling_distance,
            )
        )

        # smooth reference path
        smoothing_interface = ICurvatureSmoother()
        smoothed_reference_path = smoothing_interface.smooth(
            smoothing_strategy=smoothing_strategy,
            reference_path=resampled_reference_path,
            path_length_per_point=path_length_per_point,
        )

        # resample after smoothing
        resampled_reference_path, path_length_per_point = (
            cubic_spline_arc_interpolation_2D(polyline=smoothed_reference_path)
        )
        resampled_curvature = compute_scalar_curvature_from_polyline(
            resampled_reference_path
        )

        # curvature
        curvature_spline = CubicSpline(path_length_per_point, resampled_curvature)
        path_curvature = curvature_spline(path_length_per_point)

        # get speed limits
        speed_limits = get_speed_limits_from_lanelet_network(
            resampled_reference_path, route.lanelet_network
        )
        initial_idx = project_initial_state_on_ref_path(
            resampled_reference_path, planning_problem.initial_state
        )
        goal_idx = project_goal_on_ref_path(
            resampled_reference_path, planning_problem.goal
        )

        if initial_idx >= goal_idx:
            _logger = logging.getLogger(
                name="IVelocityPlanner.velocity_planning_problem.VppBuilder"
            )
            _logger.warning(
                "The planning problem is malformed, choosing last point of reference path as goal idx"
            )
            goal_idx = resampled_reference_path.shape[0] - 1

        if initial_idx + offset_start_idx >= goal_idx - offset_end_idx:
            _logger = logging.getLogger(
                name="IVelocityPlanner.velocity_planning_problem.VppBuilder"
            )
            _logger.warning(
                "Accounting for offsets, v_min constraint: initial_idx + offset_start >= goal_idx - offset_end cannot be computed"
            )

        return VelocityPlanningProblem(
            planning_problem=planning_problem,
            sampled_ref_path=resampled_reference_path,
            sampled_start_idx=initial_idx,
            sampled_goal_idx=goal_idx,
            interpoint_distance=interpoint_distance,
            path_length_per_point=path_length_per_point,
            path_curvature=path_curvature,
            speed_limits=speed_limits,
            v_initial=route.initial_state.velocity,
            v_stop=get_goal_velocity(
                goal_region=planning_problem.goal,
                default_velocity=default_goal_velocity,
            ),
            a_initial=route.initial_state.acceleration,
            a_stop=get_goal_acceleration(
                goal_region=planning_problem.goal,
                default_acceleration=default_goal_acceleration,
            ),
        )
