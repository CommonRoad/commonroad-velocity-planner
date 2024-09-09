import unittest
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

import commonroad_velocity_planner.fast_api as fast_api


class TestFastApi(unittest.TestCase):

    def test_fast_api(self) -> None:
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


