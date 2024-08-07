import logging

import numpy as np
from scipy.spatial.kdtree import KDTree

# Commonroad
from commonroad.planning.planning_problem import InitialState
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.scenario import Interval


def project_goal_on_ref_path(
    reference_path: np.ndarray,
    goal_region: GoalRegion,
) -> int:
    """
    Get closest point idx on reference path
    :param reference_path: reference path
    :param goal_region: cr goal region
    :return: index of closest point in reference path for goal region
    """
    if not hasattr(goal_region.state_list[0], "position"):
        # if no position is specified take last point of reference path
        goal_mid_position = reference_path[-1, :]
    elif hasattr(goal_region.state_list[0].position, "center"):
        goal_mid_position: np.ndarray = goal_region.state_list[0].position.center
    else:
        # For uncertain position route planner takes first polygon
        goal_mid_position: np.ndarray = (
            goal_region.state_list[0].position.shapes[0].center
        )
        _logger = logging.getLogger(name="IVelocityPlanner.utils.planning_problem")
        _logger.info(
            "For uncertain goal positions velocity planner takes center first polygon"
        )

    return project_point_on_ref_path(
        reference_path=reference_path, point=goal_mid_position
    )


def project_initial_state_on_ref_path(
    reference_path: np.ndarray,
    initial_state: InitialState,
) -> int:
    """
    Gets idx of closest point on ref path to initial state.
    :param reference_path: reference path
    :param initial_state: cr initial state
    :return: index of closest point of reference path to initial state
    """

    return project_point_on_ref_path(
        reference_path=reference_path, point=initial_state.position
    )


def project_point_on_ref_path(reference_path: np.ndarray, point: np.ndarray) -> int:
    """
    Project point on reference path
    :param reference_path: reference path as np.ndarray
    :param point: np.ndarray point
    :return: idx of point
    """
    kd_tree = KDTree(reference_path)
    _, idx = kd_tree.query(point)

    return idx


def get_goal_velocity(
    goal_region: GoalRegion, default_velocity: float = 0.0, min_velocity: float = 0.0
) -> float:
    """
    Get goal velocity.
    :param goal_region: cr goal region
    :param min_velocity: minimum goal velocity. Some cr scenarios are negative, which might cause infeasibility problems
    :return: goal velocity as float
    """
    if hasattr(goal_region.state_list[0], "velocity"):
        # take lower value of velocity intervall
        if isinstance(goal_region.state_list[0].velocity, Interval):
            retval: float = goal_region.state_list[0].velocity.start
        else:
            retval: float = goal_region.state_list[0].velocity
    else:
        # For uncertain position route planner takes first polygon
        _logger = logging.getLogger(name="IVelocityPlanner.utils.planning_problem")
        _logger.info(
            "For uncertain goal positions velocity planner takes center first polygon"
        )
        retval = default_velocity

    return max(min_velocity, retval)


def get_goal_acceleration(
    goal_region: GoalRegion, default_acceleration: float = 0.0
) -> float:
    """
    Get goal acceleration.
    :param goal_region: cr goal region
    :return: goal acceleration as float
    """
    if hasattr(goal_region.state_list[0], "acceleration"):
        retval: float = goal_region.state_list[0].acceleration
    else:
        # For uncertain position route planner takes first polygon
        _logger = logging.getLogger(name="IVelocityPlanner.utils.planning_problem")
        _logger.info(
            "For uncertain goal positions velocity planner takes center first polygon"
        )
        retval: float = default_acceleration

    return retval
