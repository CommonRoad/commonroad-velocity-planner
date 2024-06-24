import math

import numpy as np

# third party
import cvxpy as cp

# own code base
from commonroad_velocity_planner.config.velocity_config import VelocityConfig

# typing
from typing import Tuple, Any


class CVXPYSolver:
    """
    QP Solver using cvxpy
    """
    def __init__(self,
                 num_steps: int,
                 solver: str = cp.CLARABEL
    ) -> None:
        self._num_steps:int = num_steps
        self._v_square = cp.Variable(num_steps, nonneg=True) # is b in paper
        self._acceleration = cp.Variable(num_steps) # is a in paper
        self._objective = 0.0
        self._constraints = []
        self._solver = solver





    def solve(self,
              debug=True) -> np.ndarray:
        """
        solves the lp
        :return: Tuple [v^2, _acceleration]
        """
        prob = cp.Problem(cp.Minimize(self._objective), self._constraints)
        prob.solve(solver=self._solver)

        if(prob.solution.status == cp.INFEASIBLE):
            raise Exception(
                f"{prob.solution.status}"
            )

        if(debug):
            print(np.array([it.value for it in self._acceleration]))
            print(np.array([math.sqrt(it.value) for it in self._v_square]))

        return np.array([math.sqrt(it.value) for it in self._v_square])

    def add_velocity_to_objective(self,
                                  ) -> None:
        """
        Add velocity to objective
        """
        self._objective = cp.sum(- self._v_square)



    def add_pseudo_jerk_to_objective(
        self,
        smooth_weight: float,
        velocity_config: VelocityConfig
    ) -> None:
        """
        Add pseudo jerk
        :param smooth_weight: Adds smoothing weight
        :param param:
        """
        for i in range(self._num_steps - 1):
            self._objective += (
                    smooth_weight * (self._acceleration[i + 1] - self._acceleration[i]) ** 2 / velocity_config.distances[i]
            )

    def add_hard_velocity_constraint(self,
                                     max_velocity_array: np.ndarray,
                                     terminal_idx: int
                                     ) -> None:
        """
        Adds velocity constrained, unless for terminals state. Equivalent to (17bc)
        :param max_velocities: (n,) array of velocity constraints
        """
        self._constraints.extend(
            [self._v_square[i] <= max_velocity_array[i] ** 2 for i in range(self._num_steps) if (i != terminal_idx)]
        )



    def add_hard_acceleration_constraint(self,
                                         a_max: float,
                                         a_min: float
                                         ) -> None:
        """
        Add hard acceleration constrained. Constraind (17d) from paper
        :param a_max: maximal positive acceleration
        :param a_min: minimal acceleration (maxmimum braking)
        """
        self._constraints.extend(
            [a_min <= self._acceleration[i] for i in range(self._num_steps)]
        )

        self._constraints.extend(
            [self._acceleration[i] <= a_max for i in range(self._num_steps)]
        )


    def add_hard_jerk_constraint(
        self, velocity_config: VelocityConfig, vel_approx: np.ndarray
    ) -> None:
        self._constraints.extend(
            [
                velocity_config.j_min * velocity_config.distances[i]
                <= (self._acceleration[i + 1] - self._acceleration[i]) * vel_approx[i]
                for i in range(self._num_steps - 1)
            ]
            + [
                (self._acceleration[i + 1] - self._acceleration[i]) * vel_approx[i]
                <= velocity_config.j_max * velocity_config.distances[i]
                for i in range(self._num_steps - 1)
            ]
        )


    def add_dynamics_as_constraint(self,
                               interpoint_distances: np.ndarray,
                               ) -> None:
        """
        Adds constrained as dynamics. constrained (17b) from paper.
        :param interpoint_distances: delta arclength
        """

        # constrained (17b) from paper
        self._constraints.extend(
            [
                self._v_square[i + 1] - self._v_square[i] == 2 * self._acceleration[i] * interpoint_distances[i]
                for i in range(self._num_steps - 1)
            ]
        )

    def add_initial_constraints(self,
                                v_initial: float,
                                a_initial: float
                                ) -> None:
        self._constraints.extend(
            [
                self._v_square[0] == v_initial ** 2,
                self._acceleration[0] == a_initial,
            ]
        )


    def add_terminal_constraints(self,
                                v_terminal: float,
                                a_terminal: float,
                                terminal_idx: int
                                ) -> None:

            self._constraints.extend(
                [
                    self._v_square[terminal_idx] == v_terminal ** 2,
                    self._acceleration[terminal_idx] == a_terminal,
                ]
            )

    def const_constrained(self, var: Any) -> np.ndarray:
        """
        Generates an array of length n for given input
        :param var: variable to be put into an array
        :return: (n,) np.ndarray
        """
        return np.asarray([var for _ in range(self._num_steps)])
