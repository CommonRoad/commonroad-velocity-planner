# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad_route_planner.route_planner import (
    RoutePlanner,
    RouteGenerator,
)
from commonroad_route_planner.route import Route

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

    # route planner
    route_planner = RoutePlanner(
        lanelet_network=scenario.lanelet_network,
        planning_problem=planning_problem,
    )
    route_generator: RouteGenerator = route_planner.plan_routes()
    route: Route = route_generator.retrieve_shortest_route()

    # Velocity Planner config
    velocity_planner_config: (
        VelocityPlannerConfig
    ) = ConfigurationBuilder().get_predefined_configuration(
        planner_config=PlannerConfig.DEFAULT,
    )

    # velocity planning problem
    vpp: VelocityPlanningProblem = VppBuilder().build_vpp(
        route=route,
        planning_problem=planning_problem,
        resampling_distance=2.0,
        default_goal_velocity=planning_problem.initial_state.velocity,
        smoothing_strategy=SmoothingStrategy.ELASTIC_BAND,
    )

    # Velocity Planner
    vpi = IVelocityPlanner()

    return vpi.plan_velocity(
        route=route,
        planner_config=velocity_planner_config,
        velocity_planning_problem=vpp,
    )
