import enum
from dataclasses import dataclass
from enum import Enum

import cvxpy as cp

# typing
from typing import Optional


@enum.unique
class ConstraintType(Enum):
    HARD = 0
    SOFT_LINEAR = 1
    SOFT_QUADRATIC = 2


@enum.unique
class VelMaxType(Enum):
    # no normalization
    PLAIN = 0
    # normalization to v_max from config
    SCALED_TO_MAX_VEL = 1
    # scale to jerk-filtered algorithm
    SCALED_TO_APPROX_VEL = 2


@enum.unique
class JerkMinType(Enum):
    NONE = 0
    PSEUDO_JERK = 1
    APPROXIMATED_JERK = 2


@enum.unique
class VelBoundType(Enum):
    # max vel from config
    MAX_VEL_BOUNDED = 0
    # from jerk filtered
    APPROX_VEL_BOUNDED = 1


@enum.unique
class SolverBackend(Enum):
    CLARABEL = cp.CLARABEL
    GUROBI = cp.GUROBI


@dataclass(frozen=True)
class OptimizationConfig:
    """
    Configuration used in optimization-based planners
    """

    # normalize velocity to prevent ueberschwingen
    velocity_maximization_type: VelMaxType
    # activating jerk minization
    jerk_minimization_type: JerkMinType
    jerk_min_weight: float
    velocity_bound_type: VelBoundType
    velocity_constraint: ConstraintType
    velocity_over_weight: float
    acceleration_constraint: ConstraintType
    acceleration_over_weight: float
    approximated_jerk_constraint: Optional[ConstraintType]
    approximated_jerk_over_weight: float
    pseudo_jerk_constraint: Optional[ConstraintType]
    pseudo_jerk_over_weight: float
    solver: SolverBackend
