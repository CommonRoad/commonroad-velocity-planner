import unittest
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.route import Route
from commonroad_velocity_planner.velocity_planner_interface import IVelocityPlanner
from commonroad_velocity_planner.configuration.configuration_builder import ConfigurationBuilder
from commonroad_velocity_planner.velocity_planning_problem import VppBuilder

class TestMinimalExamle(unittest.TestCase):

    def test_minimal_example(self):
        # File
        path_scenario = Path(__file__).parents[1] / "scenarios" / "DEU_GarchingCampus2D-2.xml"

        # cr-io
        scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        # route planner
        route: Route = RoutePlanner(
            lanelet_network=scenario.lanelet_network, planning_problem=planning_problem
        ).plan_routes().retrieve_shortest_route()

        # Velocity Planner
        global_trajectory = IVelocityPlanner().plan_velocity(
            route=route,
            planner_config=ConfigurationBuilder().get_predefined_configuration(),
            velocity_planning_problem=VppBuilder().build_vpp(
                route=route,
                planning_problem=planning_problem,
                default_goal_velocity=planning_problem.initial_state.velocity
            )
        )

