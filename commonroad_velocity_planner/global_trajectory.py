from dataclasses import dataclass
import logging

import numpy as np
from commonroad_route_planner.reference_path import ReferencePath
from scipy.spatial import KDTree

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.planning.planning_problem import InitialState
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
from typing import List, Tuple


@dataclass
class GlobalTrajectory:
    """
    Output class, containing the reference path, with velocity and acceleration profile as well as all information
    of the CR reference_path it was generated with.
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

    def get_closest_idx(self, point: np.ndarray) -> int:
        """
        Get idx of closest point on global trajectory
        :param point: (2,) numpy array
        :return: index
        """
        kd_tree: KDTree = KDTree(self.reference_path)
        _, idx = kd_tree.query(point)
        return idx

    def get_closest_point(self, point: np.ndarray) -> Tuple[int, np.ndarray]:
        """
        Get idx and coords of closest point on global trajectory
        :param point:
        :return:
        """
        idx: int = self.get_closest_idx(point)
        return (idx, self.reference_path[idx])

    def get_velocity_at_position_with_lookahead(
        self, position: np.ndarray, lookahead_s: float = 2.0
    ) -> float:
        """
        Get velocity at position. Uses closest point of global trajectory
        :param position: (2,) position array
        :return: velocity
        """
        idx_0: int = self.get_closest_idx(position)
        v_0: float = self.velocity_profile[idx_0]
        s_0: float = self.path_length_per_point[idx_0]

        for idx in range(self.velocity_profile[idx_0:].shape[0]):
            idx_1 = idx_0 + idx
            v_1: float = self.velocity_profile[idx_1]
            s_1: np.ndarray = self.path_length_per_point[idx_1]

            delta_s = s_1 - s_0
            delta_v = v_0 + (v_1 - v_0) / 2

            if delta_v == 0:
                delta_t: float = delta_s / v_0
            else:
                delta_t: float = (s_1 - s_0) / (v_0 + (v_1 - v_0) / 2)

            if delta_t >= lookahead_s:
                break

        return self.velocity_profile[idx_1]


def factory_from_reference_path_and_velocity_profile(
    reference_path: ReferencePath,
    velocity_planning_problem: VelocityPlanningProblem,
    velocity_profile: SplineProfile,
) -> GlobalTrajectory:
    """
    Factory method from cr reference_path and velocity profile.
    :param reference_path:
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
        reference_path=reference_path.reference_path, point=vpp_start_point
    )
    route_goal_idx: int = project_point_on_ref_path(
        reference_path=reference_path.reference_path, point=vpp_end_point
    )
    arclength_to_start: float = reference_path.path_length_per_point[route_start_idx]

    velocity_array: np.ndarray = velocity_profile.interpolate_velocity_at_arc_lenth(
        reference_path.path_length_per_point - arclength_to_start
    )
    acceleration_array: np.ndarray = (
        velocity_profile.interpolate_acceleration_at_arc_lenth(
            reference_path.path_length_per_point - arclength_to_start
        )
    )

    # set velocity and acceleration before start and after goal manually
    velocity_array[:route_start_idx] = (
        np.ones_like(velocity_array[:route_start_idx]) * velocity_array[route_start_idx]
    )
    acceleration_array[:route_start_idx] = np.ones_like(
        acceleration_array[:route_start_idx] * acceleration_array[route_start_idx]
    )
    velocity_array[route_goal_idx:] = (
        np.ones_like(velocity_array[route_goal_idx:]) * velocity_array[route_goal_idx]
    )
    acceleration_array[route_goal_idx:] = (
        np.ones_like(acceleration_array[route_goal_idx:])
        * acceleration_array[route_goal_idx]
    )

    average_velocity: float = np.average(velocity_array, axis=0)
    maximum_velocity: float = np.max(velocity_array)
    minimum_velocity: float = np.min(velocity_array)

    return GlobalTrajectory(
        lanelet_network=reference_path.lanelet_network,
        initial_state=reference_path.initial_state,
        lanelet_ids=reference_path.lanelet_ids,
        sections=reference_path.sections,
        prohibited_lanelet_ids=reference_path.prohibited_lanelet_ids,
        lane_change_method=reference_path.lane_change_method,
        reference_path=reference_path.reference_path,
        num_lane_change_actions=reference_path.num_lane_change_actions,
        velocity_profile=velocity_array,
        acceleration_profile=acceleration_array,
        interpoint_distance=reference_path.interpoint_distances,
        path_length_per_point=reference_path.path_length_per_point,
        path_orientation=reference_path.path_orientation,
        path_curvature=reference_path.path_curvature,
        length_reference_path=reference_path.length_reference_path,
        average_velocity=average_velocity,
        maximum_velocity=maximum_velocity,
        minimum_velocity=minimum_velocity,
        planning_problem_start_idx=route_start_idx,
        planning_problem_goal_idx=route_goal_idx,
    )
