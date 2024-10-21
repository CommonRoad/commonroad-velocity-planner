The subsequent code snippet shows the general usage of the CommonRoad Velocity Planner

```Python
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner
import commonroad_route_planner.fast_api.fast_api as rfapi
from commonroad_route_planner.reference_path import ReferencePath
from commonroad_velocity_planner.velocity_planner_interface import IVelocityPlanner, ImplementedPlanners
from commonroad_velocity_planner.configuration.configuration_builder import ConfigurationBuilder
from commonroad_velocity_planner.velocity_planning_problem import VppBuilder

# cr-io
scenario, planning_problem_set = CommonRoadFileReader("path/to/commonroad/xml").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

# reference_path planner
reference_path: ReferencePath = rfapi.generate_reference_path_from_scenario_and_planning_problem(
    scenario=scenario,
    planning_problem=planning_problem
)

###### Planner switching #######
# You can switch the planner using the velocity_planner argument and the ImplementedPlanners enum.
# Using linear program Planner
global_trajectory_lp = IVelocityPlanner().plan_velocity(
    velocity_planner=ImplementedPlanners.LinearProgramPlanner,
    reference_path=reference_path,
    planner_config=ConfigurationBuilder().get_predefined_configuration(),
    velocity_planning_problem=VppBuilder().build_vpp(
        reference_path=reference_path,
        planning_problem=planning_problem,
        default_goal_velocity=planning_problem.initial_state.velocity,
    )
)

# Using Bang-Bang Planner
global_trajectory_bb = IVelocityPlanner().plan_velocity(
    velocity_planner=ImplementedPlanners.BangBangSTPlanner,
    reference_path=reference_path,
    planner_config=ConfigurationBuilder().get_predefined_configuration(),
    velocity_planning_problem=VppBuilder().build_vpp(
        reference_path=reference_path,
        planning_problem=planning_problem,
        default_goal_velocity=planning_problem.initial_state.velocity,
    )
)


# Using QP Planner
global_trajectory_qp = IVelocityPlanner().plan_velocity(
    velocity_planner=ImplementedPlanners.QPPlanner,
    reference_path=reference_path,
    planner_config=ConfigurationBuilder().get_predefined_configuration(),
    velocity_planning_problem=VppBuilder().build_vpp(
        reference_path=reference_path,
        planning_problem=planning_problem,
        default_goal_velocity=planning_problem.initial_state.velocity,
    )
)


```