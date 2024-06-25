from dataclasses import dataclass

import numpy as np

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.planning_problem import InitialState
from commonroad_route_planner.route import Route

# own code base
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import (
    LaneChangeMethod,
)

# typing
from typing import List, Optional


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


def raceline_factory_from_route(
        route: Route,
        velocity_profile: np.ndarray
) -> RaceLine:
    average_velocity: float = np.average(velocity_profile, axis=0),
    maximum_velocity: float = np.max(velocity_profile)
    minimum_velocity: float = np.min(velocity_profile)


    return RaceLine(
        lanelet_network=route.lanelet_network,
        initial_state=route.initial_state,
        lanelet_ids=route.lanelet_ids,
        sections=route.sections,
        prohibited_lanelet_ids=route.prohibited_lanelet_ids,
        lane_change_method=route.lane_change_method,
        reference_path=route.reference_path,
        num_lane_change_actions=route.num_lane_change_actions,
        velocity_profile=velocity_profile,
        interpoint_distance=route.interpoint_distances,
        path_length_per_point=route.path_length_per_point,
        path_orientation=route.path_orientation,
        path_curvature=route.path_curvature,
        length_reference_path=route.length_reference_path,
        average_velocity=average_velocity,
        maximum_velocity=maximum_velocity,
        minimum_velocity=minimum_velocity
    )



















