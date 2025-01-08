from commonroad_route_planner.reference_path import ReferencePathThe subsequent code snippet shows the general usage of the CommonRoad Velocity Planner

## Default Configuration

```Python
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.route import Route
from commonroad_velocity_planner.velocity_planner_interface import IVelocityPlanner
from commonroad_velocity_planner.velocity_planning_problem import VppBuilder
from commonroad_velocity_planner.configuration.configuration_builder import ConfigurationBuilder
from commonroad_velocity_planner.configuration.optimization_config import OptimizationConfig
from commonroad_velocity_planner.configuration.vehicle_config import VehicleConfig
from commonroad_velocity_planner.configuration.velocity_planner_config import VelocityPlannerConfig

# cr-io
scenario, planning_problem_set = CommonRoadFileReader("path/to/commonroad/xml").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

# reference_path planner
route: Route = RoutePlanner(
    lanelet_network=scenario.lanelet_network, planning_problem=planning_problem
).plan_routes().retrieve_shortest_route()

###### Default Configuration Building #######
# You need to build a config for optimization, vehicle and velocity planner. You can choose between
# the default configs (as shown here) and suplementing individual configs with your own config.
# Confer the documentation of the Configuration Builder to see methods for custom configs
optimization_config: OptimizationConfig = ConfigurationBuilder.build_default_optimization_config()

vehicle_config: VehicleConfig = ConfigurationBuilder.build_default_vehicle_config()

velocity_planner_config: VelocityPlannerConfig = ConfigurationBuilder.build_default_velocity_planner_config(
    optimization_config=optimization_config,
    vehicle_config=vehicle_config
)

global_trajectory = IVelocityPlanner().plan_velocity(
    reference_path=route,
    planner_config=velocity_planner_config,
    velocity_planning_problem=VppBuilder().build_vpp(
        reference_path=route,
        planning_problem=planning_problem,
        default_goal_velocity=planning_problem.initial_state.velocity,
    )
)
```

## Custom Configurations

```Python
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_route_planner.fast_api.fast_api as rfapi
from commonroad_route_planner.reference_path import ReferencePath
from commonroad_velocity_planner.velocity_planner_interface import IVelocityPlanner
from commonroad_velocity_planner.configuration.configuration_builder import ConfigurationBuilder
from commonroad_velocity_planner.configuration.vehicle_config import VehicleConfig
from commonroad_velocity_planner.configuration.velocity_planner_config import VelocityPlannerConfig
from commonroad_velocity_planner.velocity_planning_problem import VppBuilder
from commonroad_velocity_planner.configuration.optimization_config import (
    OptimizationConfig,
    ConstraintType,
    VelMaxType,
    VelBoundType,
    JerkMinType,
    SolverBackend,
)

# cr-io
scenario, planning_problem_set = CommonRoadFileReader("path/to/scenario").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

# reference_path planner
reference_path: ReferencePath = rfapi.generate_reference_path_from_scenario_and_planning_problem(
    scenario=scenario,
    planning_problem=planning_problem
)

###### Custom Configuration Building #######
# You can also specify custom configs
optimization_config: OptimizationConfig = ConfigurationBuilder.build_optimization_config(
        velocity_maximization_type=VelMaxType.SCALED_TO_APPROX_VEL,
        jerk_minimization_type=JerkMinType.APPROXIMATED_JERK,
        jerk_min_weight=30.0,
        velocity_bound_type=VelBoundType.APPROX_VEL_BOUNDED,
        velocity_constraint=ConstraintType.SOFT_QUADRATIC,
        velocity_over_weight=10e5,
        acceleration_constraint=ConstraintType.SOFT_QUADRATIC,
        acceleration_over_weight=6 * 10e6,
        approximated_jerk_constraint=None,
        approximated_jerk_over_weight=5 * 10e5,
        pseudo_jerk_constraint=ConstraintType.SOFT_QUADRATIC,
        pseudo_jerk_over_weight=5 * 10e5,
        smoothness_weight=30,
        time_weight=30,
        solver=SolverBackend.CLARABEL,
)

vehicle_config: VehicleConfig = ConfigurationBuilder.build_vehicle_config(
        length_rear=1.644,
        length_front=1.484,
        mass=2520,
        inertia_z=13600,
        tire_linear=0.3,
        tire_B_front=10,
        tire_C_front=1.3,
        tire_D_front=1.2,
        tire_B_rear=10,
        tire_C_rear=1.6,
        tire_D_rear=2.1,
)

velocity_planner_config: VelocityPlannerConfig = ConfigurationBuilder.build_velocity_config(
        optimization_config=optimization_config,
        vehicle_config=vehicle_config,
        a_lateral_max=2.0,
        a_long_comfort=0.9,
        a_min=-2.5,
        a_max=1.5,
        j_min=-4.0,
        j_max=3.6,
        v_min_driving=3.0,
        v_max_street=130.0 / 3.6,
)

global_trajectory = IVelocityPlanner().plan_velocity(
    reference_path=reference_path,
    planner_config=velocity_planner_config,
    velocity_planning_problem=VppBuilder().build_vpp(
        reference_path=reference_path,
        planning_problem=planning_problem,
        default_goal_velocity=planning_problem.initial_state.velocity,
    )
)
```