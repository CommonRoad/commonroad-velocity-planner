import unittest
from pathlib import Path

import numpy as np


# commonroad
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_route_planner.fast_api.fast_api as fapi
from commonroad_route_planner.reference_path import ReferencePath

from commonroad_velocity_planner.global_trajectory import GlobalTrajectory
# own code base
from commonroad_velocity_planner.utils.visualization.visualize_velocity_planner import visualize_global_trajectory
from commonroad_velocity_planner.utils.visualization.visualize_quantities import (
    visualize_velocity_over_arclength,
    visualize_acceleration_over_arclength,
)
from commonroad_velocity_planner.velocity_planner_interface import IVelocityPlanner, ImplementedPlanners
from commonroad_velocity_planner.preprocessing.curvature_smoother import SmoothingStrategy
from commonroad_velocity_planner.configuration.configuration_builder import ConfigurationBuilder, PlannerConfig
from commonroad_velocity_planner.configuration.velocity_planner_config import VelocityPlannerConfig
from commonroad_velocity_planner.velocity_planning_problem import VelocityPlanningProblem, VppBuilder

#typing
from typing import Tuple

class TestLookAhead(unittest.TestCase):
    """
    Test look-ahead in global trajectory.
    """


    def test_lookahead(self):

        # path scenario
        path_scenario = Path(__file__).parents[1] / "scenarios" / "ZAM_handling1-1_1_T-1.xml"

        # cr-io
        scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

        # reference_path planner
        reference_path: ReferencePath = fapi.generate_reference_path_from_scenario_and_planning_problem(
            scenario=scenario,
            planning_problem=planning_problem
        )
        # Velocity Planner config
        velocity_planner_config: VelocityPlannerConfig = ConfigurationBuilder().get_predefined_configuration(
            planner_config=PlannerConfig.DEFAULT,
        )

        # velocity planning problem
        vpp: VelocityPlanningProblem = VppBuilder().build_vpp(
            reference_path=reference_path,
            planning_problem=planning_problem,
            resampling_distance=2.0,
            default_goal_velocity=planning_problem.initial_state.velocity,
            smoothing_strategy=SmoothingStrategy.ELASTIC_BAND,
        )

        # Velocity Planner
        vpi = IVelocityPlanner()
        global_trajectory: GlobalTrajectory = vpi.plan_velocity(
            reference_path=reference_path,
            planner_config=velocity_planner_config,
            velocity_planning_problem=vpp,
            velocity_planner=ImplementedPlanners.BangBangSTPlanner
        )

        # get closest point
        idx: int = global_trajectory.get_closest_idx(
            point=planning_problem.initial_state.position
        )

        _, position = global_trajectory.get_closest_point(
            point=planning_problem.initial_state.position
        )

        velocity: float = global_trajectory.get_velocity_at_position_with_lookahead(
            position=planning_problem.initial_state.position
        )

        # TODO: functional evaluation

