from commonroad_route_planner.reference_path import ReferencePathThe subsequent code snippet shows the general usage of the CommonRoad Velocity Planner

```Python
import time
from pathlib import Path
import os

# commonroad
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_route_planner.fast_api.fast_api as rfapi
from commonroad_route_planner.reference_path import ReferencePath

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
from commonroad_velocity_planner.utils.regulatory_elements import get_regulatory_elements_position_on_path


def main(
        path_to_xml: str,
        output_dir_path: str,
        test: bool = False,
        save_img: bool = False,
        planner: ImplementedPlanners = ImplementedPlanners.LinearProgramPlanner,
) -> None:
        # cr-io
    scenario, planning_problem_set = CommonRoadFileReader(path_to_xml).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # reference_path planner
    reference_path: ReferencePath = rfapi.generate_reference_path_from_scenario_and_planning_problem(
        scenario=scenario,
        planning_problem=planning_problem
    )

    # get regulatory element position
    stop_positions = get_regulatory_elements_position_on_path(
        lanelet_ids=reference_path.lanelet_ids,
        reference_path=reference_path.reference_path,
        lanelet_network=scenario.lanelet_network,
        current_time_step=0
    )

    t_0 = time.perf_counter()
    # Velocity Planner config
    velocity_planner_config: VelocityPlannerConfig = ConfigurationBuilder().get_predefined_configuration(
        planner_config=PlannerConfig.DEFAULT,
    )

    # velocity planning problem
    vpp: VelocityPlanningProblem = VppBuilder().build_vpp(
        reference_path=reference_path,
        planning_problem=planning_problem,
        resampling_distance=2.0,
        stop_positions=stop_positions,
        default_goal_velocity=planning_problem.initial_state.velocity,
        smoothing_strategy=SmoothingStrategy.ELASTIC_BAND,
    )

    # Velocity Planner
    vpi = IVelocityPlanner()
    global_trajectory, spline_profile = vpi.plan_velocity(
        reference_path=reference_path,
        planner_config=velocity_planner_config,
        velocity_planning_problem=vpp,
        velocity_planner=planner,
        return_spline_profile=True,
    )

    print(f"velocity planning took {time.perf_counter() - t_0}")

    if not test:
        save_img = save_img
        output_path = (
                Path(output_dir_path)
                / path_to_xml.split("/")[-1].split(".")[0]
        )
        output_path.mkdir(parents=True, exist_ok=True)
        visualize_global_trajectory(
            scenario=scenario,
            velocity_planning_problem=vpp,
            global_trajectory=global_trajectory,
            save_img=save_img,
            save_path=str(output_path / "route_vel.png"),
            test=test,
        )
        visualize_velocity_over_arclength(
            path_length_per_point=global_trajectory.path_length_per_point,
            velocity_profile=global_trajectory.velocity_profile,
            v_max=15,
            v_min=0,
            save_img=save_img,
            save_path=str(output_path / "vel_over_arc.png"),
            test=test,
        )
        visualize_acceleration_over_arclength(
            path_length_per_point=global_trajectory.path_length_per_point,
            acceleration_profile=global_trajectory.acceleration_profile,
            a_max=vpi.config.a_max,
            a_min=vpi.config.a_min,
            save_img=save_img,
            save_path=str(output_path / "acc_over_arc.png"),
            test=test,
        )


if __name__ == "__main__":
    scenarios = "path/to/scenario/folder"
    output_dir_path: str = "path/to/output/dir"
    for xml in sorted(os.listdir(scenarios)):
        _xml = scenarios + "/" + xml
        main(path_to_xml=_xml, save_img=True, test=False, output_dir_path=output_dir_path)

```