from dataclasses import dataclass

import numpy as np

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.planning_problem import InitialState

# own code base
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import (
    LaneChangeMethod,
)

# typing
from typing import List



@dataclass
class RaceLine:
    """
    Class containing the reference path information with _acceleration velocity profile
    """
    lanelet_network: LaneletNetwork
    initial_state: InitialState

    lanelet_ids: List[int]
    sections: List[LaneletSection]
    prohibited_lanelet_ids: List[int]

    lane_change_method: LaneChangeMethod
    num_lane_change_actions: int

    reference_path: np.ndarray
    velocity_profile: np.ndarray
    interpoint_distance: np.ndarray
    path_length_per_point: np.ndarray
    path_orientation: np.ndarray
    path_curvature: np.ndarray
    length_reference_path: float
    average_velocity: float
    maximum_velocity: float
    minimum_velocity: float






















