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
from commonroad_velocity_planner.utils.jerk_filter import filter_jerk
from commonroad_velocity_planner.utils.max_velocity_calc import (
    calculate_lat_acc_limited_velocity,
)

# typing
from typing import Optional, Union


class LinearProgramPlanner(BaseVelocityPlanner):
    """
    Linear program planner. Note that several constraint relaxations make this a quadratic program as well.

    References:
    ----------
    - Shimizu, Y., Horibe, T., Watanabe, F., & Kato, S. (2022): Jerk constrained velocity planning
    for an autonomous vehicle: Linear programming approach.
    In 2022 International Conference on Robotics and Automation (ICRA) (pp. 5814-5820). IEEE.

    - https://github.com/pflab-ut/jerk_optimal_velocity_planning


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
        self._v_approx = filter_jerk(
            problem=problem, config=self._config, v_max=self._v_max
        )

        # set up planner interface
        self._solver_interface.update_planning_problem(
            config=self._config,
            velocity_planning_problem=problem,
            v_max=self._v_max,
            v_approx=self._v_approx,
        )

        # Add objective
        self._solver_interface.add_velocity_to_objective()
        self._solver_interface.add_jerk_to_objective()

        # Add constraints
        self._solver_interface.add_dynamic_constraint()
        self._solver_interface.add_velocity_constraint()
        self._solver_interface.add_acceleration_constraint()
        self._solver_interface.add_approximated_jerk_constraint()
        self._solver_interface.add_pseudo_jerk_constraint()
        self._solver_interface.add_planning_problem_constraints(
            initial_idx=problem.sampled_start_idx, goal_idx=problem.sampled_goal_idx
        )
        self._solver_interface.add_v_min_constraint(
            initial_idx=problem.sampled_start_idx,
            goal_idx=problem.sampled_goal_idx,
            v_min_driving=self._config.v_min_driving,
        )

        # solve optimization problem
        return self._solver_interface.solve()
