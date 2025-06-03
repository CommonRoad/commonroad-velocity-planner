from dataclasses import dataclass

from scipy.spatial.kdtree import KDTree
from typing import List, Set

import numpy as np
from commonroad.scenario.scenario import Scenario, Lanelet
from commonroad.scenario.traffic_light import (
    TrafficLight,
    TrafficLightState,
)


@dataclass
class StopPosition:
    """
    Dataclass for index and color of traffic light
    """

    stop_line_position: np.ndarray


def get_regulatory_elements_position_on_path(
    lanelet_ids: List[int],
    scenario: Scenario,
    reference_path: np.ndarray,
    current_time_step: int = 0,
    th_lane_change: float = 1.0,
) -> List[StopPosition]:
    """
    Find indices of regulatory elements on reference path
    :param lanelet_ids: lanelet ids of route
    :param reference_path: (n,2) np.ndarray of reference path points
    :param scenario: CR Scenario
    :return: list of path traffic light indices
    """
    stop_positions: List[StopPosition] = list()
    kd_tree: KDTree = KDTree(reference_path)

    for lanelet_id in lanelet_ids:
        lanelet: Lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
        traffic_light_ids: Set[int] = lanelet.traffic_lights
        if len(traffic_light_ids) > 0:
            # TODO: check which traffic light is actually the right one
            traffic_light: TrafficLight = (
                scenario.lanelet_network.find_traffic_light_by_id(
                    list(traffic_light_ids)[0]
                )
            )
            if traffic_light.active:
                if traffic_light.traffic_light_cycle.active:
                    if (
                        traffic_light.traffic_light_cycle.get_state_at_time_step(
                            current_time_step
                        )
                        == TrafficLightState.RED
                    ):
                        # closest point
                        last_point_of_centerline: np.ndarray = lanelet.center_vertices[
                            -1
                        ]
                        _, idx_closest_point = kd_tree.query(last_point_of_centerline)
                        closest_point_on_path: np.ndarray = reference_path[
                            idx_closest_point
                        ]

                        if (
                            np.linalg.norm(
                                closest_point_on_path - last_point_of_centerline
                            )
                            < th_lane_change
                        ):
                            stop_positions.append(
                                StopPosition(
                                    stop_line_position=last_point_of_centerline,
                                )
                            )

            elif lanelet.stop_line is not None:
                stop_positions.append(
                    StopPosition(
                        stop_line_position=lanelet.stop_line.start,
                    )
                )

    return stop_positions
