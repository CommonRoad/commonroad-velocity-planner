####################################################
#
# Adapted from Shimizu et al.: Jerk Velocity
# Planning for an Autonomous Vehicle. Linear Programming Approach
#
#
#
#################################################

import numpy as np
import cvxpy as cpy

# own code base
from commonroad_velocity_planner.optimizer.cvxpy_optimizer import CVXPYSolver
from commonroad_velocity_planner.config.velocity_config import VelocityConfig
from commonroad_velocity_planner.planners.jerk_filter import filter_jerk
from commonroad_velocity_planner.util.visualization import visualize_route_with_velocity
from commonroad_velocity_planner.util.goal_state import find_idx_of_terminal_state

# typing
from typing import Union


class LinearProgramWithJerkFilter:
    """
    This velocity planner uses Linear Programming for optimizing the velocity. The approach is described in
    equation system (17) in [1]. The main idea is to approximate the optimal velocity profile and use this
    in order to linearize the jerk constraint.

    Definition of the problem:
        variables:
            x = [b[0], b[1], ..., b[N - 1] | a[0], a[1], .... a[N - 1]]
        with:
            b[i]: velocity ** 2
            a[i]: acceleration

        minimization objective:
            -sum(b[i] / (max_velocities[i] ** 2))

        constraints:
            (b[i+1] - b[i]) / ds = 2a[i]                                        "Dynamic Constraint"

            0 < b[i] < max_velocities[i] ** 2                                   "Velocity Constraint"
            a_min < a[i] < a_max                                                "Acceleration Constraint"
            j_min / vel_approx[i] < pseudo_jerk[i] < j_max / vel_approx[i]      "Jerk Constraint"

            b[0] = v_initial ** 2                                               "Boundary Constraint"
            b[N - 1] = v_stop ** 2
            a[0] = a_initial
            a[N - 1] = a_stop

    [1] Shimizu, Yutaka, et al. "Jerk constrained velocity planning for an autonomous vehicle: Linear
        programming approach." 2022 International Conference on Robotics and Automation (ICRA). IEEE, 2022.

    """


    def __init__(self,
                 velocity_config: VelocityConfig,
                 max_velocity: Union[float, np.ndarray],
                 solver=cpy.CLARABEL,
                 ) -> None:

        self._velocity_config: VelocityConfig = velocity_config


        self._solver = solver
        self._optimizer = CVXPYSolver(
            num_steps=self._velocity_config.num_steps,
            solver=solver
        )

        # construct max velocity
        self._max_velocity: np.ndarray = max_velocity if(
            isinstance(max_velocity, np.ndarray)
        ) else self._optimizer.const_constrained(max_velocity)

        self._init_optimizer()




    def _init_optimizer(self) -> None:
        """
        Inits optimizer by adding constraints
        """

        self._optimizer.add_velocity_to_objective()

        self._optimizer.add_initial_constraints(
            v_initial=self._velocity_config.v_initial,
            a_initial=self._velocity_config.a_initial
        )


        terminal_idx: int = find_idx_of_terminal_state(
            planning_problem=planning_problem,
            reference_path=route.reference_path
        )

        self._optimizer.add_terminal_constraints(
            v_terminal=self._velocity_config.v_terminal,
            a_terminal=self._velocity_config.a_terminal,
            terminal_idx=terminal_idx
        )

        self._optimizer.add_hard_velocity_constraint(
            max_velocity=self._velocity_config.v_max,
            terminal_idx=terminal_idx
        )

        self._optimizer.add_hard_acceleration_constraint(
            a_max=self._velocity_config.a_max,
            a_min=self._velocity_config.a_min
        )

        self._optimizer.add_dynamics_as_constraint(
            interpoint_distances=self._velocity_config.distances
        )




        #self._optimizer.add_hard_acceleration_constraint(self._velocity_config)
        #vel_approx = filter_jerk(self._velocity_config, self._max_velocity)
        #self._optimizer.add_hard_jerk_constraint(self._velocity_config, vel_approx)
        #self._optimizer.add_dynamic_constraint(self._velocity_config)
        #self._optimizer.add_boundary_constraint(self._velocity_config)






    def plan_velocity(self) -> np.ndarray:
        """
        Plans velocity profile
        :return: (n,) np.ndarray of velocities
        """
        return self._optimizer.solve()





if __name__ == "__main__":
    from commonroad_route_planner.route_planner import RoutePlanner
    from commonroad.common.file_reader import CommonRoadFileReader
    from commonroad_velocity_planner.config.factory_methods import create_velocity_config_from_route_and_planning_problem

    scenario, planning_problem_set = CommonRoadFileReader(f"/home/tmasc/projects/velocity_planner/commonroad-velocity-planner/scenarios/ZAM_Turorial-1_2_T-1_modified.xml").open()
    # retrieve the first planning problem in the problem set
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    route = RoutePlanner(
        scenario=scenario,
        planning_problem=planning_problem
    ).plan_routes().retrieve_shortest_route()

    velocity_config = create_velocity_config_from_route_and_planning_problem(
        planning_problem=planning_problem,
        route=route,
        v_terminal = 15,
        a_terminal = 0,
        v_max = 30,
        a_lateral_max= 1,
        a_min = -4,
        a_max = 4,
        j_min = -4,
        j_max = 4,
    )

    planner = LinearProgramWithJerkFilter(
        velocity_config=velocity_config,
        max_velocity=velocity_config.v_max
    )

    result = planner.plan_velocity()
    print(result)

    visualize_route_with_velocity(
        route=route,
        velocity_array=result,
        scenario=scenario,
        planning_problem=planning_problem,
        save_img=False
    )










