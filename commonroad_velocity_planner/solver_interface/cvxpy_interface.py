import time
import logging

# third party
import numpy as np
import cvxpy as cp

# own code base
from commonroad_velocity_planner.configuration.optimization_config import (
    ConstraintType,
    VelBoundType,
    JerkMinType,
    VelMaxType,
)
from commonroad_velocity_planner.configuration.velocity_planner_config import (
    VelocityPlannerConfig,
)
from commonroad_velocity_planner.spline_profile import SplineProfile
from commonroad_velocity_planner.velocity_planning_problem import (
    VelocityPlanningProblem,
)

# TODO: Make much prettier
# TODO: Deconvolute interface


class CvxpyInterface:
    """
    cxpy interface
    """

    def __init__(self, solver: str = cp.CLARABEL) -> None:
        """
        Base _solver interface
        :param solver: cvxpy _solver
        """
        super().__init__()
        self._logger = logging.getLogger(
            name="IVelocityPlanner.solver_interface.cvxpy_interface"
        )

        self._solver = solver

        self._objective = 0.0
        self._constraints = []

        self._config: VelocityPlannerConfig = None
        self._problem: VelocityPlanningProblem = None

        self._n: int = None
        self._b: cp.Variable = None
        self._a: cp.Variable = None

        self._v_max: np.ndarray = None
        self._v_approx: np.ndarray = None

    @property
    def problem(self) -> VelocityPlanningProblem:
        """
        :return: velocity planning problem
        """
        return self._problem

    @property
    def config(self) -> VelocityPlannerConfig:
        """
        :return: Velocity planner config
        """
        return self._config

    def update_planning_problem(
        self,
        config: VelocityPlannerConfig,
        velocity_planning_problem: VelocityPlanningProblem,
        v_max: np.ndarray,
        v_approx: np.ndarray,
    ) -> None:
        """
        Updates planning problem
        :param config:
        :param velocity_planning_problem:
        :param v_max:
        :param v_approx:
        :return:
        """
        self._config = config
        self._problem = velocity_planning_problem

        self._objective = 0.0
        self._constraints = []

        self._n = velocity_planning_problem.path_curvature.shape[0]
        self._b = cp.Variable(self._n, nonneg=True)
        self._a = cp.Variable(self._n)

        self._v_max = v_max
        self._v_approx = v_approx

    def add_velocity_to_objective(self) -> None:
        """
        Adds velocity to objective as specified in config
        """
        if (
            self._config.optimization_config.velocity_maximization_type
            == VelMaxType.SCALED_TO_MAX_VEL
        ):
            self._objective -= cp.sum(self._b / np.square(self._v_max).clip(min=0.01))
        elif (
            self._config.optimization_config.velocity_maximization_type
            == VelMaxType.SCALED_TO_APPROX_VEL
        ):
            self._objective -= cp.sum(
                self._b / np.square(self._v_approx).clip(min=0.01)
            )
        else:
            self._objective -= cp.sum(self._b)

    def add_jerk_to_objective(self) -> None:
        """
        Adds jerk to objective as specified in config
        """
        ds = self._problem.interpoint_distance
        if (
            self._config.optimization_config.jerk_minimization_type
            == JerkMinType.PSEUDO_JERK
        ):
            for i in range(self._n - 1):
                self._objective += (
                    self._config.optimization_config.jerk_min_weight
                    * ((self._a[i + 1] - self._a[i]) / ds) ** 2
                )
        elif (
            self._config.optimization_config.jerk_minimization_type
            == JerkMinType.APPROXIMATED_JERK
        ):
            for i in range(self._n - 1):
                approx_vel = 0.5 * (self._v_approx[i] + self._v_approx[i + 1])
                self._objective += (
                    self._config.optimization_config.jerk_min_weight
                    * ((self._a[i + 1] - self._a[i]) * approx_vel / ds) ** 2
                    * ds
                )

    def add_dynamic_constraint(self) -> None:
        """
        Adds dynamic dipendencies between jerk, aceleration and velocity
        """
        ds = self._problem.interpoint_distance
        self._constraints.extend(
            [
                self._b[i + 1] - self._b[i] == 2 * self._a[i] * ds
                for i in range(self._n - 1)
            ]
        )

    def add_boundary_constraint(self) -> None:
        """
        Add boundary constraints for first and last point
        """
        self._constraints.extend(
            [
                self._b[0] == self._problem.v_initial**2,
                self._b[-1] == self._problem.v_stop**2,
                self._a[0] == self._problem.a_initial,
                self._a[-1] == self._problem.a_stop,
            ]
        )

        # relaxed goal _constraints
        slack_var_b_pos = cp.Variable(1, nonneg=True)
        slack_var_b_neg = cp.Variable(1, nonneg=True)
        w_b = 5 * 10e5

        slack_var_a_pos = cp.Variable(1, nonneg=True)
        slack_var_a_neg = cp.Variable(1, nonneg=True)
        w_a = 5 * 10e5

        self._objective += w_b * cp.sum(slack_var_b_pos + slack_var_b_neg)
        self._constraints.extend(
            [
                self._b[-1] + slack_var_b_pos <= self._problem.v_stop**2,
                self._b[-1] - slack_var_b_neg >= self._problem.v_stop**2,
            ]
        )

        self._objective += w_a * cp.sum(slack_var_a_pos + slack_var_a_neg)
        self._constraints.extend(
            [
                self._a[-1] + slack_var_a_pos <= self._problem.a_stop,
                self._a[-1] - slack_var_a_neg >= self._problem.a_stop,
            ]
        )

    def add_velocity_constraint(self) -> None:
        """
        Add velocity constraints as specified in config
        """
        if (
            self._config.optimization_config.velocity_bound_type
            == VelBoundType.MAX_VEL_BOUNDED
        ):
            velocity_bound = self._v_max
        else:
            velocity_bound = self._v_approx

        if self._config.optimization_config.velocity_constraint == ConstraintType.HARD:
            self._constraints.extend(
                [self._b[i] <= velocity_bound[i] ** 2 for i in range(1, self._n - 1)]
            )
        elif self._config.optimization_config.velocity_constraint in [
            ConstraintType.SOFT_LINEAR,
            ConstraintType.SOFT_QUADRATIC,
        ]:
            vel_slack_var = self.add_slack_var(
                self._config.optimization_config.velocity_constraint,
                self._config.optimization_config.velocity_over_weight,
            )
            self._constraints.extend(
                [
                    self._b[i] - vel_slack_var[i] <= velocity_bound[i] ** 2
                    for i in range(1, self._n - 1)
                ]
            )

    def add_acceleration_constraint(self) -> None:
        """
        Add acceleration constraints as specified in config
        """
        if (
            self._config.optimization_config.acceleration_constraint
            == ConstraintType.HARD
        ):
            self._constraints.extend(
                [self._config.a_min <= self._a[i] for i in range(1, self._n - 1)]
                + [self._a[i] <= self._config.a_max for i in range(1, self._n - 1)]
            )
        elif self._config.optimization_config.acceleration_constraint in [
            ConstraintType.SOFT_LINEAR,
            ConstraintType.SOFT_QUADRATIC,
        ]:
            acc_slack_var = self.add_slack_var(
                self._config.optimization_config.acceleration_constraint,
                self._config.optimization_config.acceleration_over_weight,
            )
            self._constraints.extend(
                [
                    self._config.a_min <= self._a[i] - acc_slack_var[i]
                    for i in range(1, self._n - 1)
                ]
                + [
                    self._a[i] - acc_slack_var[i] <= self._config.a_max
                    for i in range(1, self._n - 1)
                ]
            )

    def add_approximated_jerk_constraint(self) -> None:
        """
        Add approximated jerk constrained
        """
        ds = self._problem.interpoint_distance
        if (
            self._config.optimization_config.approximated_jerk_constraint
            == ConstraintType.HARD
        ):
            self._constraints.extend(
                [
                    self._config.j_min * ds
                    <= (self._a[i + 1] - self._a[i]) * self._v_approx[i]
                    for i in range(self._n - 1)
                ]
                + [
                    (self._a[i + 1] - self._a[i]) * self._v_approx[i]
                    <= self._config.j_max * ds
                    for i in range(self._n - 1)
                ]
            )
        elif self._config.optimization_config.approximated_jerk_constraint in [
            ConstraintType.SOFT_LINEAR,
            ConstraintType.SOFT_QUADRATIC,
        ]:
            jerk_slack_var = self.add_slack_var(
                self._config.optimization_config.approximated_jerk_constraint,
                self._config.optimization_config.approximated_jerk_over_weight,
            )
            self._constraints.extend(
                [
                    self._config.j_min * ds
                    <= (self._a[i + 1] - self._a[i]) * self._v_approx[i]
                    - jerk_slack_var[i] * ds
                    for i in range(self._n - 1)
                ]
                + [
                    (self._a[i + 1] - self._a[i]) * self._v_approx[i]
                    - jerk_slack_var[i] * ds
                    <= self._config.j_max * ds
                    for i in range(self._n - 1)
                ]
            )

    def add_pseudo_jerk_constraint(self) -> None:
        """
        Add pseudo jerk constraint as specified in config
        """
        ds = self._problem.interpoint_distance
        if (
            self._config.optimization_config.approximated_jerk_constraint
            == ConstraintType.HARD
        ):
            self._constraints.extend(
                [
                    self._config.j_min * ds <= (self._a[i + 1] - self._a[i])
                    for i in range(self._n - 1)
                ]
                + [
                    (self._a[i + 1] - self._a[i]) <= self._config.j_max * ds
                    for i in range(self._n - 1)
                ]
            )
        elif self._config.optimization_config.pseudo_jerk_constraint in [
            ConstraintType.SOFT_LINEAR,
            ConstraintType.SOFT_QUADRATIC,
        ]:
            jerk_slack_var = self.add_slack_var(
                self._config.optimization_config.pseudo_jerk_constraint,
                self._config.optimization_config.pseudo_jerk_over_weight,
            )
            self._constraints.extend(
                [
                    self._config.j_min * ds
                    <= (self._a[i + 1] - self._a[i]) - jerk_slack_var[i] * ds
                    for i in range(self._n - 1)
                ]
                + [
                    (self._a[i + 1] - self._a[i]) - jerk_slack_var[i] * ds
                    <= self._config.j_max * ds
                    for i in range(self._n - 1)
                ]
            )

    def add_planning_problem_constraints(self, initial_idx: int, goal_idx: int) -> None:
        """
        Adds planning problem as constrained
        :param initial_idx: index of initial state
        :param goal_idx: index of goal state
        """
        self._constraints.extend(
            [
                self._b[: initial_idx + 1]
                == np.ones_like(range(initial_idx + 1)) * self._problem.v_initial**2,
                self._a[: initial_idx + 1]
                == np.ones_like(range(initial_idx + 1)) * self._problem.a_initial,
            ]
        )

        # relaxed goal _constraints
        slack_var_b_pos = cp.Variable(self._b[: initial_idx + 1].shape[0], nonneg=True)
        slack_var_b_neg = cp.Variable(self._b[: initial_idx + 1].shape[0], nonneg=True)
        w_b = 5 * 10e5

        slack_var_a_pos = cp.Variable(self._a[goal_idx:].shape[0], nonneg=True)
        slack_var_a_neg = cp.Variable(self._a[goal_idx:].shape[0], nonneg=True)
        w_a = 5 * 10e5

        self._objective += w_b * cp.sum(slack_var_b_pos + slack_var_b_neg)
        self._constraints.extend(
            [
                self._b[: initial_idx + 1] + slack_var_b_pos
                <= np.ones_like(range(initial_idx + 1)) * self._problem.v_initial**2,
                self._b[: initial_idx + 1] - slack_var_b_neg
                >= np.ones_like(range(initial_idx + 1)) * self._problem.v_initial**2,
            ]
        )

        self._objective += w_a * cp.sum(slack_var_a_pos + slack_var_a_neg)
        self._constraints.extend(
            [
                self._a[goal_idx:] + slack_var_a_pos
                <= np.ones_like(range(goal_idx, self._n)) * self._problem.a_stop,
                self._a[goal_idx:] - slack_var_a_neg
                >= np.ones_like(range(goal_idx, self._n)) * self._problem.a_stop,
            ]
        )

    def add_v_min_constraint(
        self,
        initial_idx: int,
        goal_idx: int,
        v_min_driving: float,
        offset_goal: int = 3,
        offset_start: int = 2,
    ) -> None:
        """
        Add minimum driving velocity constraint. This velocity must be kept at all times between initial_idx + offset start
        and goal_idx - offset_goal
        :param initial_idx: initial state idx
        :param goal_idx: goal state idx
        :param v_min_driving: minimum driving velocity
        :param offset_goal: goal offset index
        :param offset_start: initial state offset index
        :return:
        """
        if initial_idx + offset_start >= goal_idx - offset_goal:
            self._logger.warning(
                f"V_min constrained not possible with offsets: initial_idx={initial_idx}, goal_idx={goal_idx}"
            )
            offset_goal = 0
            offset_start = 1

        self._constraints.extend(
            [
                self._b[initial_idx + offset_start : goal_idx - offset_goal]
                >= np.ones_like(
                    self._b[initial_idx + offset_start : goal_idx - offset_goal]
                )
                * (v_min_driving**2)
            ]
        )

    def add_slack_var(
        self, constraint_type: ConstraintType, weight: float
    ) -> cp.Variable:
        """
        Create slack variable
        :param constraint_type: either soft linear or soft quadratic
        :param weight: weight
        :return: cvxpy variable
        """
        if constraint_type == ConstraintType.SOFT_LINEAR:
            return self.add_linear_slack_var(weight)
        elif constraint_type == ConstraintType.SOFT_QUADRATIC:
            return self.add_quadratic_slack_var(weight)

    def add_linear_slack_var(self, weight: float) -> cp.Variable:
        """
        Add linear slack variable
        :param weight: weight
        :return: cvxpy variable
        """
        slack_var_pos = cp.Variable(self._n, nonneg=True)
        slack_var_neg = cp.Variable(self._n, nonneg=True)
        self._objective += weight * sum(slack_var_pos + slack_var_neg)
        return slack_var_pos - slack_var_neg

    def add_quadratic_slack_var(self, weight: float) -> cp.Variable:
        """
        Generate quadratic slack variable
        :param weight: weight
        :return: cvxpy variable
        """
        slack_var = cp.Variable(self._n)
        for i in range(self._n):
            self._objective += weight * slack_var[i] ** 2
        return slack_var

    def solve(self) -> SplineProfile:
        """
        solve problem
        :return: spline profile object
        """
        prob = cp.Problem(cp.Minimize(self._objective), self._constraints)

        t_0 = time.perf_counter()

        prob.solve(solver=self._solver, verbose=False)
        self._logger.info(f"solving took {time.perf_counter() - t_0}")
        self._logger.info(f"solver status: {prob.solution.status}")

        if prob.solution.status == cp.INFEASIBLE:
            self._logger.error(
                "Wasn't able to solve velocity optimization "
                "(probably due to infeasible self._constraints, for example impossible initial velocity or acceleration)"
            )
            raise Exception(
                "Wasn't able to solve velocity optimization "
                "(probably due to infeasible self._constraints, for example impossible initial velocity or acceleration)"
            )

        return SplineProfile(
            path_length_per_point=self._problem.path_length_per_point,
            velocity_profile=np.sqrt(self._b.value).clip(min=0),
            acceleration_profile=self._a.value,
            goal_velocity=self._problem.v_stop,
        )
