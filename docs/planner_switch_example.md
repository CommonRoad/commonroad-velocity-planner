The subsequent code snippet shows the general usage of the CommonRoad Velocity Planner


```Python
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.route import Route
from commonroad_velocity_planner.velocity_planner_interface import IVelocityPlanner, ImplementedPlanners
from commonroad_velocity_planner.configuration.configuration_builder import ConfigurationBuilder
from commonroad_velocity_planner.velocity_planning_problem import VppBuilder

# cr-io
scenario, planning_problem_set = CommonRoadFileReader("path/to/commonroad/xml").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

# route planner
route: Route = RoutePlanner(
    lanelet_network=scenario.lanelet_network, planning_problem=planning_problem
).plan_routes().retrieve_shortest_route()



 ###### Planner switching #######
 # You can switch the planner using the velocity_planner argument and the ImplementedPlanners enum.
# Using linear program Planner
global_trajectory_lp = IVelocityPlanner().plan_velocity(
    velocity_planner=ImplementedPlanners.LinearProgramPlanner,
    route=route,
    planner_config=ConfigurationBuilder().get_predefined_configuration(),
    velocity_planning_problem=VppBuilder().build_vpp(
        route=route, 
        planning_problem=planning_problem, 
        default_goal_velocity=planning_problem.initial_state.velocity,
    )
)

# Using Bang-Bang Planner
global_trajectory_bb = IVelocityPlanner().plan_velocity(
    velocity_planner=ImplementedPlanners.BangBangSTPlanner,
    route=route,
    planner_config=ConfigurationBuilder().get_predefined_configuration(),
    velocity_planning_problem=VppBuilder().build_vpp(
        route=route, 
        planning_problem=planning_problem, 
        default_goal_velocity=planning_problem.initial_state.velocity,
    )
)

```