

import numpy as np

# own code base
from commonroad_velocity_planner.config.velocity_config import VelocityConfig


# typing
from typing import List, Tuple

def filter_jerk_forward(
    v0: float,
    a0: float,
    a_max: float,
    j_max: float,
    ds: np.ndarray,
    velocities: np.ndarray,
) -> np.ndarray:


    current_vel: float = v0
    current_acc: float = a0

    filtered_velocities = np.zeros_like(velocities)
    current_vel, current_acc = apply_limits(
        v_limit=velocities[0],
        v=current_vel,
        a=current_acc
    )
    filtered_velocities[0] = current_vel

    for i in range(1, len(filtered_velocities)):
        # It takes max_dt time to travel ds[i - 1] with a constant jerk of j_max
        # and a starting velocity and acceleration of 0
        dt = ds[i] / max(current_vel, 1.0e-6)

        if current_acc + j_max * dt >= a_max:
            # Prevent the acceleration from overshooting a_max
            tmp_jerk = min((a_max - current_acc) / dt, j_max)
            current_vel = current_vel + current_acc * dt + 0.5 * tmp_jerk * dt * dt
            current_acc = a_max
        else:
            current_vel = current_vel + current_acc * dt + 0.5 * j_max * dt * dt
            current_acc = current_acc + j_max * dt

        current_vel, current_acc = apply_limits(
            v_limit=velocities[i],
            v=current_vel,
            a=current_acc
        )
        filtered_velocities[i] = current_vel

    return filtered_velocities


def filter_jerk_backward(
    v_end: float,
    a_end: float,
    a_min: float,
    j_min: float,
    ds: np.ndarray,
    velocities: np.ndarray,
) -> np.ndarray:
    rev_vel = filter_jerk_forward(
        v_end, a_end, abs(a_min), abs(j_min), np.flip(ds), np.flip(velocities[1:,])
    )
    return np.flip(rev_vel)


def filter_jerk(config: VelocityConfig,
                max_velocities: np.ndarray
                ) -> np.ndarray:
    """
    Filters jerk given max velocity array
    :param config: velocity config
    :param max_velocities:
    :return: (n,) np.ndarray of floats for filtered jerk
    """
    forward_velocities = filter_jerk_forward(
        config.v_initial,
        config.a_initial,
        config.a_max,
        config.j_max,
        config.distances,
        max_velocities,
    )
    return filter_jerk_backward(
        config.v_stop,
        config.a_stop,
        config.a_min,
        config.j_min,
        config.distances,
        forward_velocities,
    )



def apply_limits(v_limit: float,
                 v: float,
                 a: float
                 ) -> Tuple[float, float]:
    """
    Apply velocity limits
    :param v_limit: velocity limit
    :param v: current velocity
    :param a: current acceleration
    :return: Tuple[velocity, acceleration]
    """
    ep = 1.0e-5
    if v > v_limit + ep:
        v = v_limit
        a = 0.0
    if v < 0.0:
        v = 0.0
        a = 0.0
    return (v, a)



