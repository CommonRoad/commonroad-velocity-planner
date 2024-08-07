import numpy as np

from commonroad_velocity_planner.velocity_planning_problem import (
    VelocityPlanningProblem,
)
from commonroad_velocity_planner.configuration.velocity_planner_config import (
    VelocityPlannerConfig,
)


def filter_jerk_forward(
    v0: float,
    a0: float,
    a_max: float,
    j_max: float,
    ds: float,
    velocities: np.ndarray,
) -> np.ndarray:
    """
    Forward part of Jerk filter as implemented by Shimizu et al. in their github repo
    :param v0: starting velocity
    :param a0: starting acceleration
    :param a_max: maximum acceleration
    :param j_max: maximum jerk
    :param ds: average interpoint arc length
    :param velocities: velocity array (n,)
    :return: filtered velocity array (n,)
    """
    # TODO: Currently only uses average ds. However, when combined with elastic band smoothing, the ds varies

    # TODO: Move to outer part
    def apply_limits(v, a, i):
        v_lim = velocities[i]
        ep = 1.0e-5
        if v > v_lim + ep:
            v = v_lim
            a = 0.0
        if v < 0.0:
            v = 0.0
            a = 0.0
        return v, a

    current_vel = v0
    current_acc = a0

    filtered_velocities = np.zeros_like(velocities)
    current_vel, current_acc = apply_limits(current_vel, current_acc, 0)
    filtered_velocities[0] = current_vel

    for i in range(1, len(filtered_velocities)):
        # It takes max_dt time to travel ds[i - 1] with a constant jerk of j_max
        # and a starting velocity and acceleration of 0
        max_dt = (6 * ds / j_max) ** (1 / 3)
        dt = min(ds / max(current_vel, 1.0e-6), max_dt)

        if current_acc + j_max * dt >= a_max:
            # Prevent the acceleration from overshooting a_max
            tmp_jerk = min((a_max - current_acc) / dt, j_max)
            current_vel = current_vel + current_acc * dt + 0.5 * tmp_jerk * dt * dt
            current_acc = a_max
        else:
            current_vel = current_vel + current_acc * dt + 0.5 * j_max * dt * dt
            current_acc = current_acc + j_max * dt

        current_vel, current_acc = apply_limits(current_vel, current_acc, i)
        filtered_velocities[i] = current_vel

    return filtered_velocities


def filter_jerk_backward(
    v_end: float,
    a_end: float,
    a_min: float,
    j_min: float,
    ds: float,
    velocities: np.ndarray,
) -> np.ndarray:
    """
    Backward jerk filter pass as implemented by Shimizu et al. in their github repo.
    :param v_end: end velocity
    :param a_end: end acceleration
    :param a_min: minimum acceleration
    :param j_min: minimum jerk
    :param ds: average interpoint arc length
    :param velocities: velocity array
    :return: jerk filtered velocity array
    """
    # TODO: Currently only uses average ds. However, when combined with elastic band smoothing, the ds varies

    rev_vel = filter_jerk_forward(
        v_end, a_end, abs(a_min), abs(j_min), ds, np.flip(velocities)
    )
    return np.flip(rev_vel)


def filter_jerk(
    problem: VelocityPlanningProblem, config: VelocityPlannerConfig, v_max: np.ndarray
) -> np.ndarray:
    """
    Jerk filter as implemented by Shimizu et al. in their github repo: https://github.com/pflab-ut/jerk_optimal_velocity_planning
    :param problem:
    :param config:
    :param v_max:
    :return:
    """

    forward_velocities = filter_jerk_forward(
        problem.v_initial,
        problem.a_initial,
        config.a_max,
        config.j_max,
        problem.interpoint_distance,
        v_max,
    )
    v_stop = problem.v_stop
    a_stop = problem.a_stop
    return filter_jerk_backward(
        v_stop,
        a_stop,
        config.a_min,
        config.j_min,
        problem.interpoint_distance,
        forward_velocities,
    )
