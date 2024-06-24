import math

import numpy as np

# commonroad
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem

# own code base
from commonroad_velocity_planner.race_line import RaceLine
from commonroad_velocity_planner.util.parking_vehicles_heuristic import heuristically_identify_if_vehicle_on_street_is_parking


# typing
from typing import List




class VelocityBehaviorPlanner:
    """
    Limits the velocity profile given the scenario to a reasonable value.
    """

    def __init__(
            self,
            vehicle_length: float,
            vehicle_width: float,
    ) -> None:
        # vehicle params
        self._vehicle_width = vehicle_width
        self._vehicle_length = vehicle_length




    def plan_velocity(
            self,
            scenario: Scenario,
            velocity_profile: np.ndarray,
            vehicle_position_x: float,
            vehicle_position_y: float,
            vehicle_velocity_x: float,
            vehicle_velocity_y: float,
            current_speed_limit: float = 10,
            lateral_threshold: float = 0.5,
            parking_pass_speed_ms: float = 2,
            look_ahead_sec: float=10
    ) -> np.ndarray:
        """
        Updates velocity profile given parking vehicles
        :param scenario:
        :param race_line:
        :param vehicle_position_x:
        :param vehicle_position_y:
        :param vehicle_velocity_x:
        :param vehicle_velocity_y:
        :param current_speed_limit:
        :param lateral_threshold:
        :param parking_pass_speed_ms:
        :param look_ahead_sec:
        :return: velocity profile
        """

        # look ahead in m
        look_ahead_m: float = look_ahead_sec * min(
            math.sqrt(vehicle_velocity_x**2 + vehicle_velocity_y**2), current_speed_limit
        )

        # hard cut on maximum currently detected speed limit
        velocity_profile.clip(a_min=0, a_max=current_speed_limit)

        # for all parking vehicles in radius, check the distance left for driving of ego vehicle. If this distance
        # is below threshold, reduce speed
        distance_list: List[float] = [np.inf]
        for dyn_obs in scenario.dynamic_obstacles:
            if(
                    math.sqrt(
                        (dyn_obs.initial_state.position[0] - vehicle_position_x)**2
                        + (dyn_obs.initial_state.position[1] - vehicle_position_y)**2)
                    > look_ahead_m
            ):
                continue

            parking, distance_obs = heuristically_identify_if_vehicle_on_street_is_parking(
                dyn_obstacle=dyn_obs,
                lanelet_network=scenario.lanelet_network,
            )
            if(parking):
                distance_list.append(distance_obs)

        # clipp to parking pass distance
        smallest_distance_parking: float = min(distance_list)
        if(smallest_distance_parking < lateral_threshold):
            velocity_profile.clip(a_min=0, a_max=parking_pass_speed_ms)


        return velocity_profile

















