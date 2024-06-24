from dataclasses import dataclass

import numpy as np

@dataclass(frozen=True)
class VelocityConfig:
    """
    Velocity Config data class
    """

    reference_path: np.ndarray
    path_curvature: np.ndarray
    distances: np.ndarray
    num_steps: int

    v_initial: float
    v_terminal: float
    a_initial: float
    a_terminal: float

    v_max: float
    a_min: float
    a_max: float
    a_lateral_max: float
    j_min: float
    j_max: float



