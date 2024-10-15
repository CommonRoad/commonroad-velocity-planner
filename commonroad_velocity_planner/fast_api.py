# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.scenario import Scenario
import commonroad_route_planner.fast_api.fast_api as fapi
from commonroad_route_planner.reference_path import ReferencePath

# own code base
from commonroad_velocity_planner.global_trajectory import GlobalTrajectory
from commonroad_velocity_planner.velocity_planner_interface import IVelocityPlanner
from commonroad_velocity_planner.preprocessing.curvature_smoother import (
    SmoothingStrategy,
)
from commonroad_velocity_planner.configuration.configuration_builder import (
    ConfigurationBuilder,
    PlannerConfig,
)
from commonroad_velocity_planner.configuration.velocity_planner_config import (
    VelocityPlannerConfig,
)
from commonroad_velocity_planner.velocity_planning_problem import (
    VelocityPlanningProblem,
    VppBuilder,
)


def global_trajectory_from_scenario_and_planning_problem(
    scenario: Scenario,
    planning_problem: PlanningProblem,
) -> GlobalTrajectory:
    """
    Get global trajectory from scenario and planning problem
    :param scenario: CommonRoad scenario
    :param planning_problem: CommonRoad planning problem
    :return: CommonRoad global trajectory
    """
    return global_trajectory_from_lanelet_network_and_planning_problem(
        lanelet_network=scenario.lanelet_network, planning_problem=planning_problem
    )


def global_trajectory_from_lanelet_network_and_planning_problem(
    lanelet_network: LaneletNetwork,
    planning_problem: PlanningProblem,
) -> GlobalTrajectory:
    """
    Get global trajectory from lanelet network and planning problem
    :param lanelet_network: CommonRoad lanelet network
    :param planning_problem: CommonRoad planning problem
    :return: CommonRoad global trajectory
    """
    # ========== retrieving reference path =========== #
    # here we retrieve the shortest reference_path that has the least amount of disjoint lane changes
    reference_path: "ReferencePath" = (
        fapi.generate_reference_path_from_lanelet_network_and_planning_problem(
            lanelet_network=lanelet_network, planning_problem=planning_problem
        )
    )

    # Velocity Planner config
    velocity_planner_config: (
        VelocityPlannerConfig
    ) = ConfigurationBuilder().get_predefined_configuration(
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

    return vpi.plan_velocity(
        reference_path=reference_path,
        planner_config=velocity_planner_config,
        velocity_planning_problem=vpp,
    )


def global_trajectory_from_cr_reference_path_and_planning_problem(
    cr_reference_path: ReferencePath, planning_problem: PlanningProblem
) -> GlobalTrajectory:
    """
    Get global trajectory from CommonRoad reference path and planning problem
    :param cr_reference_path: CommonRoad reference path object
    :param planning_problem: CommonRoad planning problem
    :return: CommonRoad global trajectory
    """

    # Velocity Planner config
    velocity_planner_config: (
        VelocityPlannerConfig
    ) = ConfigurationBuilder().get_predefined_configuration(
        planner_config=PlannerConfig.DEFAULT,
    )

    # velocity planning problem
    vpp: VelocityPlanningProblem = VppBuilder().build_vpp(
        reference_path=cr_reference_path,
        planning_problem=planning_problem,
        resampling_distance=2.0,
        default_goal_velocity=planning_problem.initial_state.velocity,
        smoothing_strategy=SmoothingStrategy.ELASTIC_BAND,
    )

    # Velocity Planner
    vpi = IVelocityPlanner()

    return vpi.plan_velocity(
        reference_path=cr_reference_path,
        planner_config=velocity_planner_config,
        velocity_planning_problem=vpp,
    )
