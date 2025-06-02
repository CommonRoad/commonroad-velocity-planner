import unittest
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.reference_path import ReferencePath
import commonroad_route_planner.fast_api.fast_api as route_fapi

import commonroad_velocity_planner.fast_api as fast_api
from commonroad_velocity_planner.velocity_planner_interface import ImplementedPlanners


class TestFastApi(unittest.TestCase):

    def test_fast_api_from_scenario(self) -> None:
        """
        Tests fast api
        """

        path_scenario = Path(__file__).parents[1] / "scenarios" / "DEU_GarchingCampus2D-2.xml"

        # cr-io
        scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        global_trajectory = fast_api.global_trajectory_from_scenario_and_planning_problem(
            scenario=scenario,
            planning_problem=planning_problem
        )

    def test_fast_api_from_lanelet_network(self) -> None:
        """
        Tests fast api
        """

        path_scenario = Path(__file__).parents[1] / "scenarios" / "DEU_GarchingCampus2D-2.xml"

        # cr-io
        scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        global_trajectory = fast_api.global_trajectory_from_lanelet_network_and_planning_problem(
            lanelet_network=scenario.lanelet_network,
            planning_problem=planning_problem
        )

    def test_fast_api_from_ref_path(self) -> None:
        """
        Tests fast api
        """

        path_scenario = Path(__file__).parents[1] / "scenarios" / "DEU_GarchingCampus2D-2.xml"

        # cr-io
        scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        reference_path: ReferencePath = route_fapi.generate_reference_path_from_scenario_and_planning_problem(
            scenario=scenario,
            planning_problem=planning_problem
        )

        global_trajectory = fast_api.global_trajectory_from_cr_reference_path_and_planning_problem(
            cr_reference_path=reference_path,
            planning_problem=planning_problem
        )

    def test_fast_api_planner_switch(self) -> None:
        path_scenario = Path(__file__).parents[1] / "scenarios" / "DEU_GarchingCampus2D-2.xml"

        # cr-io
        scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        global_trajectory = fast_api.global_trajectory_from_scenario_and_planning_problem(
            scenario=scenario,
            planning_problem=planning_problem,
            velocity_planner=ImplementedPlanners.QPPlanner
        )

        global_trajectory = fast_api.global_trajectory_from_scenario_and_planning_problem(
            scenario=scenario,
            planning_problem=planning_problem,
            velocity_planner=ImplementedPlanners.LinearProgramPlanner
        )

        global_trajectory = fast_api.global_trajectory_from_scenario_and_planning_problem(
            scenario=scenario,
            planning_problem=planning_problem,
            velocity_planner=ImplementedPlanners.BangBangSTPlanner
        )
