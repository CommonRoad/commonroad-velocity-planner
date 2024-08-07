from dataclasses import dataclass

# own code base
from commonroad_velocity_planner.configuration.optimization_config import (
    OptimizationConfig,
)
from commonroad_velocity_planner.configuration.vehicle_config import VehicleConfig


@dataclass(frozen=True)
class VelocityPlannerConfig:
    """
    Velocity planner config, uniting optimization config, vehicle config and several top-level params.
    """

    optimization_config: OptimizationConfig
    vehicle_config: VehicleConfig
    a_min: float
    a_max: float
    a_lateral_max: float
    j_min: float
    j_max: float
    v_min_driving: float
    v_max_street: float
    g: float = 9.81
