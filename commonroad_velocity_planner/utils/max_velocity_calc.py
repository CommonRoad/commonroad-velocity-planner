import math

import numpy as np

from commonroad_velocity_planner.velocity_planning_problem import (
    VelocityPlanningProblem,
)
from commonroad_velocity_planner.configuration.velocity_planner_config import (
    VelocityPlannerConfig,
)
from commonroad_velocity_planner.utils.planning_problem import (
    project_initial_state_on_ref_path,
)


def calculate_lat_acc_limited_velocity(
    problem: VelocityPlanningProblem,
    config: VelocityPlannerConfig,
    decel_distance_before_curve=3.5,
    decel_distance_after_curve=3.5,
    min_curve_velocity=2.74,
    enable_smooth_limit=True,
    min_decel_for_lateral_acc_lim_filter=-2.5,
    v_min: float = 2.0,
    v_max: float = 130.0 / 3.6,
    initial_offset_idx: int = 1,
) -> np.ndarray:
    """
    Calculates acceleration limits of arc at each point given the curvature
    :param problem: velocity planning problem
    :param config: velocity planner config
    :param decel_distance_before_curve: decelaration distance before curve
    :param decel_distance_after_curve: acceleration distance after curve
    :param min_curve_velocity: minimum curvature velocity
    :param enable_smooth_limit: uses velocity and jerk smoother as preprocessing step
    :param min_decel_for_lateral_acc_lim_filter: minimum filter decelartion
    :param v_min: minimum driving velocity
    :param v_max: maximum velocity
    :return: velocity array (n,)
    """
    n = problem.path_curvature.shape[0]
    output = problem.speed_limits.copy()
    points_interval = problem.interpoint_distance

    lat_acc_min_vel_arr = []
    if enable_smooth_limit:
        lat_acc_min_vel_arr = (
            calc_velocity_profile_with_constant_jerk_and_acceleration_limit(
                n,
                problem.v_initial,
                problem.a_initial,
                config.j_min,
                config.a_min,
                min_decel_for_lateral_acc_lim_filter,
                problem.interpoint_distance,
            )
        )

    for i in range(n - 1):
        before_decel_index = int(round(decel_distance_before_curve / points_interval))
        after_decel_index = int(round(decel_distance_after_curve / points_interval))

        curvature = 0.0
        start = i - after_decel_index if i > after_decel_index else 0
        end = min(n, i + before_decel_index + 1)
        for j in range(start, end):
            curvature = max(curvature, abs(problem.path_curvature[j]))
        v_curvature_max = math.sqrt(config.a_lateral_max / max(curvature, 1.0e-5))
        v_curvature_max = max(v_curvature_max, min_curve_velocity)

        if enable_smooth_limit:
            if i >= len(lat_acc_min_vel_arr):
                return output
            v_curvature_max = max(v_curvature_max, lat_acc_min_vel_arr[i])
        if output[i] > v_curvature_max:
            output[i] = v_curvature_max

        output = output.clip(min=v_min, max=v_max)
        initial_idx = project_initial_state_on_ref_path(
            reference_path=problem.sampled_ref_path,
            initial_state=problem.planning_problem.initial_state,
        )

        output[: initial_idx + initial_offset_idx] = (
            problem.planning_problem.initial_state.velocity
        )

    return output


def calc_velocity_profile_with_constant_jerk_and_acceleration_limit(
    n: int,
    v0: float,
    a0: float,
    jerk: float,
    acc_max: float,
    acc_min: float,
    interpoint_distance: np.ndarray,
) -> np.ndarray:
    """
    Calculates velocity profile as preprocessing step with constant jerk and acceleration limits
    :param n: number of points
    :param v0: initial velocity
    :param a0: initial accelration
    :param jerk: flat
    :param acc_max: maximum accelartion
    :param acc_min: minimum acceleration = maximum decelartion
    :param interpoint_distance: arc length per point
    :return:
    """
    velocities = np.zeros(n)
    velocities[0] = v0
    curr_v = v0
    curr_a = a0

    for i in range(n - 1):
        t = interpoint_distance / max(velocities[i], 1.0e-5)
        curr_v = integrate_v(curr_v, curr_a, jerk, t)
        velocities[i + 1] = curr_v
        curr_a = max(min(integrate_a(curr_a, jerk, t), acc_max), acc_min)

    return velocities


def integrate_v(v0: float, a0: float, j0: float, t: float) -> float:
    """
    Integrate velocity given acceleration and jerk
    :param v0: velocity
    :param a0: acceleration
    :param j0: jerk
    :param t: delta_t
    :return: integrated velocity
    """
    return v0 + a0 * t + 0.5 * j0 * t * t


def integrate_a(a0: float, j0: float, t: float) -> float:
    """
    Integrate acceleration given jerk
    :param a0: acceleration
    :param j0: jerk
    :param t: time step
    :return: integrated acceleration
    """
    return a0 + j0 * t
