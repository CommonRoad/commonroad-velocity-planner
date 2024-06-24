import os
import copy

import matplotlib.pyplot as plt
from matplotlib.colors import rgb2hex
from matplotlib import cm
import numpy as np
from scipy.spatial.kdtree import KDTree


# commonrodad
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.draw_params import MPDrawParams


# typing
from typing import Union, List
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from commonroad_route_planner.route import Route, RouteSlice






def find_idx_of_terminal_state(
        planning_problem: PlanningProblem,
        reference_path: np.ndarray
) -> int:
    """
    Finds index of terminal state via goal of planning problem and reference path or route
    :param planning_problem:
    :param route:
    :return: idx of reference path
    """

    if hasattr(planning_problem.goal.state_list[0].position, "center"):
        goal_mid_position: np.ndarray = planning_problem.goal.state_list[
            0
        ].position.center
    else:
        # For uncertain position route planner takes first polygon
        goal_mid_position: np.ndarray = (
            planning_problem.goal.state_list[0].position.shapes[0].center
        )

    _, idx = KDTree(reference_path).query(goal_mid_position)

    return idx








