import unittest
from copy import deepcopy
from pathlib import Path
import logging

# commonroad
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.visualization import visualize_planner_at_timestep
from commonroad_rp.utility.evaluation import run_evaluation
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.logger import initialize_logger

# own code base
from commonroad_velocity_planner.velocity_planner_interface import IVelocityPlanner
from commonroad_velocity_planner.configuration.configuration_builder import ConfigurationBuilder
from commonroad_velocity_planner.velocity_planning_problem import VppBuilder


class TestReactivePlannerIntegration(unittest.TestCase):

    def test_reactive_planner_integration(self):

        working = [
            "DEU_Aachen-2_1_I-1.xml",  # only with velplan
            "DEU_Aachen-9_50_I-1.xml",
            "DEU_Aachen-9_67_I-1.xml",
            "DEU_Cologne-19_77_I-1.xml", # only with velplan
            "DEU_Cologne-19_130_I-1.xml",
            "DEU_Cologne-23_105_I-1.xml",
            "DEU_Dresden-7_17_I-1.xml", # only with velplan
            "DEU_Dresden-49_11_I-1.xml",
            "DEU_Frankfurt-3_19_I-1.xml",
            "DEU_Frankfurt-3_25_I-1.xml",
            "DEU_Muc-1_2_T-1.xml",
            "DEU_Stu-1_49_I-1.xml",
            "DEU_GarchingCampus2D-2.xml",
            "DEU_Hhr-1_1.xml",
            "DEU_Test-1_1_T-1.xml",
            "ZAM_Zip-1_6_T-1.xml",
            "USA_US101-3_4_T-1.xml",
            "USA_US101-4_3_T-1.xml",
            "USA_US101-29_1_T-1.xml",
            "USA_US101-32_1_T-1.xml"



        ]

        now_working = [
            "DEU_AAH4-0_66_T-1.xml",
            "DEU_Frankfurt-3_15_I-1.xml" # weird bug with too few time
            "DEU_Gar-3_2_T-1.xml",
            "USA_Lanker-1_2_T-1.xml",
            "USA_Lanker-2_6_T-1.xml",
            "ZAM_Tjunction-1_171_T-1.xml",
            "ZAM_Turorial-1_2_T-1_modified.xml",
            "USA_US101-22_1_T-1.xml",
            "USA_Peach-2_1_T-1.xml",
            "USA_Peach-3_1_T-1.xml",
            "USA_Peach-4_1_T-1.xml",
            "DEU_AAH4-0_66_T-1.xml",

        ]


        for filename in sorted(working[0:7]):
            with self.subTest(msg=f"Testing scenario: {filename}", filename=filename):
                self.reactive_planner_call(filename=filename)



    def reactive_planner_call(self, filename: str) -> None:
        """
        Calls for reactive planner
        :param filename: filenmane
        """
        path_scenario = Path(__file__).parents[1] / "scenarios" / filename
        path_config = Path(__file__).parents[0] / "artifacts" / "rp_config.yaml"



        # Build config object
        config = ReactivePlannerConfiguration.load(path_config, path_scenario)
        config.update()


        initialize_logger(config)
        logger = logging.getLogger("RP_LOGGER")
        logger.setLevel(logging.ERROR)

        # Route Planner
        route_planner = RoutePlanner(config.scenario.lanelet_network, config.planning_problem)
        route = route_planner.plan_routes().retrieve_first_route()
        # Velocity Planner
        global_trajectory = IVelocityPlanner().plan_velocity(
            route=route,
            planner_config=ConfigurationBuilder().get_predefined_configuration(),
            velocity_planning_problem=VppBuilder().build_vpp(
                route=route,
                planning_problem=config.planning_problem,
                default_goal_velocity=config.planning_problem.initial_state.velocity
            )
        )

        planner = ReactivePlanner(config)

        # set reference path for curvilinear coordinate system
        planner.set_reference_path(global_trajectory.reference_path)

        # **************************
        # Run Planning
        # **************************
        # Add first state to recorded state and input list
        planner.record_state_and_input(planner.x_0)

        SAMPLING_ITERATION_IN_PLANNER = True

        while not planner.goal_reached():
            current_count = len(planner.record_state_list) - 1

            # check if planning cycle or not
            plan_new_trajectory = current_count % config.planning.replanning_frequency == 0
            if plan_new_trajectory:

                # new planning cycle -> plan a new optimal trajectory
                current_position = planner.record_state_list[-1].position if planner.record_state_list else planner.x_0.position
                desired_speed: float = global_trajectory.get_velocity_at_position_with_lookahead(
                    position=current_position,
                    lookahead_s=2.0
                )

                # Change here to with or without desired velocity
                planner.set_desired_velocity(desired_velocity=desired_speed)

                if SAMPLING_ITERATION_IN_PLANNER:
                    optimal = planner.plan()
                else:
                    optimal = None
                    i = 1
                    while optimal is None and i <= planner.sampling_level:
                        optimal = planner.plan(i)

                if not optimal:
                    break

                # record state and input
                planner.record_state_and_input(optimal[0].state_list[1])

                # reset planner state for re-planning
                planner.reset(initial_state_cart=planner.record_state_list[-1],
                              initial_state_curv=(optimal[2][1], optimal[3][1]),
                              collision_checker=planner.collision_checker, coordinate_system=planner.coordinate_system)

                # visualization: create ego Vehicle for planned trajectory and store sampled trajectory set
                if config.debug.show_plots or config.debug.save_plots:
                    ego_vehicle = planner.convert_state_list_to_commonroad_object(optimal[0].state_list)
                    sampled_trajectory_bundle = None
                    if config.debug.draw_traj_set:
                        sampled_trajectory_bundle = deepcopy(planner.stored_trajectories)
            else:
                # simulate scenario one step forward with planned trajectory
                sampled_trajectory_bundle = None

                # continue on optimal trajectory
                temp = current_count % config.planning.replanning_frequency

                # record state and input
                planner.record_state_and_input(optimal[0].state_list[1 + temp])

                # reset planner state for re-planning
                planner.reset(initial_state_cart=planner.record_state_list[-1],
                              initial_state_curv=(optimal[2][1 + temp], optimal[3][1 + temp]),
                              collision_checker=planner.collision_checker, coordinate_system=planner.coordinate_system)

            #print(f"current time step: {current_count}")

            # visualize the current time step of the simulation
            if config.debug.show_plots or config.debug.save_plots:
                visualize_planner_at_timestep(scenario=config.scenario, planning_problem=config.planning_problem,
                                              ego=ego_vehicle, traj_set=sampled_trajectory_bundle,
                                              ref_path=planner.reference_path, timestep=current_count, config=config)

        # make gif
        # make_gif(config, range(0, planner.record_state_list[-1].time_step))

        # **************************
        # Evaluate results
        # **************************
        evaluate = True
        if evaluate:
            cr_solution, feasibility_list = run_evaluation(planner.config, planner.record_state_list,
                                                           planner.record_input_list)


