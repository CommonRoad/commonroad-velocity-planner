from dataclasses import dataclass
import logging

import numpy as np

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.planning_problem import InitialState
from commonroad_route_planner.route import Route
from commonroad_route_planner.route_sections.lanelet_section import LaneletSection
from commonroad_route_planner.lane_changing.lane_change_methods.method_interface import (
    LaneChangeMethod,
)

# own code base
from commonroad_velocity_planner.spline_profile import SplineProfile
from commonroad_velocity_planner.velocity_planning_problem import (
    VelocityPlanningProblem,
)
from commonroad_velocity_planner.utils.planning_problem import project_point_on_ref_path

# typing
from typing import List


@dataclass
class GlobalTrajectory:
    """
    Output class, containing the reference path, with velocity and acceleration profile as well as all information
    of the CR route it was generated with.
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
    acceleration_profile: np.ndarray
    planning_problem_start_idx: int
    planning_problem_goal_idx: int

    interpoint_distance: np.ndarray
    path_length_per_point: np.ndarray
    path_orientation: np.ndarray
    path_curvature: np.ndarray
    length_reference_path: float
    average_velocity: float
    maximum_velocity: float
    minimum_velocity: float

    def __post_init__(self):
        self.check_integrity()

    def check_integrity(self) -> None:
        """
        Checks integrity of data class
        """
        if self.minimum_velocity < 0:
            _logger = logging.getLogger(name="IVelocityPlanner.global_trajectory")
            _logger.error(
                f"Velocity profile contains entries < 0. Min val is {self.minimum_velocity}"
            )
            raise ValueError(
                f"Velocity profile contains entries < 0. Min val is {self.minimum_velocity}"
            )


def factory_from_route_and_velocity_profile(
    route: Route,
    velocity_planning_problem: VelocityPlanningProblem,
    velocity_profile: SplineProfile,
) -> GlobalTrajectory:
    """
    Factory method from cr route and velocity profile.
    :param route:
    :param velocity_planning_problem:
    :param velocity_profile:
    :return:
    """

    # As optimization is only done inside the planning problem, we have to calculate the shift to the start
    vpp_start_point: np.ndarray = velocity_planning_problem.sampled_ref_path[
        velocity_planning_problem.sampled_start_idx
    ]
    vpp_end_point: np.ndarray = velocity_planning_problem.sampled_ref_path[
        velocity_planning_problem.sampled_goal_idx
    ]
    route_start_idx: int = project_point_on_ref_path(
        reference_path=route.reference_path, point=vpp_start_point
    )
    route_goal_idx: int = project_point_on_ref_path(
        reference_path=route.reference_path, point=vpp_end_point
    )
    arclength_to_start: float = route.path_length_per_point[route_start_idx]

    velocity_array: np.ndarray = velocity_profile.interpolate_velocity_at_arc_lenth(
        route.path_length_per_point - arclength_to_start
    )
    acceleration_array: np.ndarray = (
        velocity_profile.interpolate_acceleration_at_arc_lenth(
            route.path_length_per_point - arclength_to_start
        )
    )

    # set velocity and acceleration before start and after goal manually
    velocity_array[:route_start_idx] = (
        np.ones_like(velocity_array[:route_start_idx]) * velocity_array[route_start_idx]
    )
    acceleration_array[:route_start_idx] = np.zeros_like(
        acceleration_array[:route_start_idx]
    )
    velocity_array[route_goal_idx:] = (
        np.ones_like(velocity_array[route_goal_idx:]) * velocity_array[route_goal_idx]
    )
    acceleration_array[route_goal_idx:] = np.zeros_like(
        acceleration_array[route_goal_idx:]
    )

    average_velocity: float = np.average(velocity_array, axis=0)
    maximum_velocity: float = np.max(velocity_array)
    minimum_velocity: float = np.min(velocity_array)

    return GlobalTrajectory(
        lanelet_network=route.lanelet_network,
        initial_state=route.initial_state,
        lanelet_ids=route.lanelet_ids,
        sections=route.sections,
        prohibited_lanelet_ids=route.prohibited_lanelet_ids,
        lane_change_method=route.lane_change_method,
        reference_path=route.reference_path,
        num_lane_change_actions=route.num_lane_change_actions,
        velocity_profile=velocity_array,
        acceleration_profile=acceleration_array,
        interpoint_distance=route.interpoint_distances,
        path_length_per_point=route.path_length_per_point,
        path_orientation=route.path_orientation,
        path_curvature=route.path_curvature,
        length_reference_path=route.length_reference_path,
        average_velocity=average_velocity,
        maximum_velocity=maximum_velocity,
        minimum_velocity=minimum_velocity,
        planning_problem_start_idx=route_start_idx,
        planning_problem_goal_idx=route_goal_idx,
    )
