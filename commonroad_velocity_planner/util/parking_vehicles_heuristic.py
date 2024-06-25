from enum import Enum, unique

import numpy as np
# third party
from shapely import Polygon as ShapelyPolygon
from shapely import LineString as ShapelyLine
from shapely import Point as ShapelyPoint
from shapely import distance

# commonroad
from commonroad.scenario.scenario import Scenario, LaneletNetwork, Lanelet
from commonroad.scenario.obstacle import DynamicObstacle


from typing import Tuple

@unique
class LaneletEdge(Enum):
    LEFT_EDGE = 1
    RIGHT_EDGE = 2
    CENTER = 3


def heuristically_identify_if_vehicle_on_street_is_parking(
        dyn_obstacle: DynamicObstacle,
        lanelet_network: LaneletNetwork,
        velocity_threshold_mps: float = 2
) -> Tuple[bool, float]:
    """
    Checks via heuristic, if vehicle is parking.


    Heuristic
    ---------
    - Vehicle is close to other part of lanelet
    - vehicle has slow/no velocity

    :param dyn_obstacle:
    :param lanelet_network:
    :param velocity_threshold_mps:
    :return: True, if vehicle is parking on a single lane and distance to other side
    """

    parking_flag: bool = False

    # check if partially on lanelet
    if(len(lanelet_network.find_lanelet_by_position([dyn_obstacle.initial_state.position])) == 0):
        parking_flag = True
        smallest_distance: float = np.inf
    else:
        # Check if near edge of lanelet network and low/no velocity
        lanelet: Lanelet = lanelet_network.find_lanelet_by_id(
            lanelet_network.find_lanelet_by_position([dyn_obstacle.initial_state.position])[0][0]
        )
        closest_line, smallest_distance = center_closer_to_edge(
                dyn_obstacle,
                lanelet
        )

        # 1. check if center is closer to lanelet edge then to middle line
        if(closest_line == LaneletEdge.CENTER):
            parking_flag = False

        # 2. if true, check if there is a neighboring lanenelet
        elif(
                (closest_line == LaneletEdge.LEFT_EDGE and lanelet.adj_left is not None) or
                (closest_line == LaneletEdge.RIGHT_EDGE and lanelet.adj_right is not None)
        ):
            parking_flag = False

        # 3. if false, check if velocity is small
        elif(dyn_obstacle.initial_state.velocity > velocity_threshold_mps):
            parking_flag = False

        else:
            parking_flag = True

    return parking_flag, smallest_distance


def center_closer_to_edge(
        dyn_obs: DynamicObstacle,
        lanelet: Lanelet
) -> Tuple[LaneletEdge, float]:
    """
    Check if dyn_obs is closer to edge than to center
    :param dyn_obs:
    :param lanelet:
    :return: True, if closer to edge
    """

    closest_line: LaneletEdge = None
    smallest_distance: float = None

    distance_right: float = distance(
            ShapelyPoint(dyn_obs.initial_state.position[0], dyn_obs.initial_state.position[1]),
            ShapelyLine(lanelet.right_vertices)
    )

    distance_left: float = distance(
            ShapelyPoint(dyn_obs.initial_state.position[0], dyn_obs.initial_state.position[1]),
            ShapelyLine(lanelet.left_vertices)
    )

    distance_center: float = distance(
        ShapelyPoint(dyn_obs.initial_state.position[0], dyn_obs.initial_state.position[1]),
        ShapelyLine(lanelet.center_vertices)
    )

    if(distance_right < distance_left and distance_right < distance_center):
        closest_line = LaneletEdge.RIGHT_EDGE
        smallest_distance = distance(
            ShapelyPoint(dyn_obs.initial_state.position[0], dyn_obs.initial_state.position[1]),
            ShapelyLine(lanelet.left_vertices)
        )
    elif(distance_left < distance_right and distance_left < distance_center):
        closest_line = LaneletEdge.LEFT_EDGE
        smallest_distance = distance(
            ShapelyPoint(dyn_obs.initial_state.position[0], dyn_obs.initial_state.position[1]),
            ShapelyLine(lanelet.right_vertices)
        )
    else:
        closest_line = LaneletEdge.CENTER
        smallest_distance = 0


    return closest_line, smallest_distance









