import numpy as np

# Own code base
from commonroad_velocity_planner.planner.base_planner import BaseVelocityPlanner
from commonroad_velocity_planner.configuration.velocity_planner_config import (
    VelocityPlannerConfig,
)
from commonroad_velocity_planner.solver_interface.cvxpy_interface import CvxpyInterface
from commonroad_velocity_planner.velocity_planning_problem import (
    VelocityPlanningProblem,
)
from commonroad_velocity_planner.spline_profile import SplineProfile
from commonroad_velocity_planner.utils.max_velocity_calc import (
    calculate_lat_acc_limited_velocity,
)

# typing
from typing import Optional, Union


class QPPlanner(BaseVelocityPlanner):
    """
    Quadratic program planner.

    References:
    ----------
    - Zhang, Y., Chen, H., Waslander, S. L., Yang, T., Zhang, S., Xiong, G., & Liu, K. (2018).
      Toward a more complete, flexible, and safer speed planning for autonomous driving via convex optimization.
      Sensors, 18(7).


    """

    def __init__(
        self,
        config: VelocityPlannerConfig,
    ) -> None:
        """
        Convex optimization planner
        :param config: velocity planner config
        """

        super().__init__(velocity_planner_config=config)

        self._solver_interface: CvxpyInterface = CvxpyInterface(
            solver=config.optimization_config.solver.name
        )

        self._v_max = None
        self._v_approx = None

    @property
    def solver_interface(self) -> Union[CvxpyInterface]:
        """
        :return: solver interface
        """
        return self._solver_interface

    @property
    def v_approx(self) -> np.ndarray:
        """
        :return: approximated maximum velocity
        """
        return self._v_approx

    def plan_velocity(
        self,
        problem: VelocityPlanningProblem,
        config: Optional[VelocityPlannerConfig] = None,
    ) -> SplineProfile:

        # update config
        self._config = config if config is not None else self._config

        # Calculate velocity bound
        self._v_max = calculate_lat_acc_limited_velocity(
            problem=problem, config=self._config, v_max=self._config.v_max_street
        )

        # set up planner interface
        self._solver_interface.update_planning_problem(
            config=self._config,
            velocity_planning_problem=problem,
            v_max=self._v_max,
            v_approx=self._v_max,
        )

        # - Add objectives -
        # time efficiency -> J_T
        self._solver_interface.add_time_efficiency_objective()

        # reference velocity tracking -> J_V not necessary here

        # add smoothness objective
        self._solver_interface.add_smoothness_to_objective()

        # - Add constraints -
        # velocity constraints
        self._solver_interface.add_velocity_constraint()
        self._solver_interface.add_v_min_constraint(
            initial_idx=problem.sampled_start_idx,
            goal_idx=problem.sampled_goal_idx,
            v_min_driving=self._config.v_min_driving,
        )

        # dynamic constraint
        self._solver_interface.add_dynamic_constraint()

        # Boundary conditions
        self._solver_interface.add_planning_problem_constraints(
            initial_idx=problem.sampled_start_idx, goal_idx=problem.sampled_goal_idx
        )

        # Time Window constraint -> not necessary

        # Comfort box constraints
        self._solver_interface.add_longitudinal_acceleration_constraint()
        self._solver_interface.add_longitudinal_comfort_acceleration_constraint()
        # vmax are already included in the lateral acceleration limits, reimplementation not necessary
        self._solver_interface.add_velocity_constraint()

        # solve optimization problem
        return self._solver_interface.solve()
