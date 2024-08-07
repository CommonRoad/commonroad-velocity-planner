from dataclasses import dataclass


@dataclass
class VehicleConfig:
    """
    Vehicle configuration. Used in bang bang control planners.
    """

    mass: float
    length_rear: float
    length_front: float
    inertia_z: float

    # Tire model
    tire_B_front: float
    tire_C_front: float
    tire_D_front: float
    tire_B_rear: float
    tire_C_rear: float
    tire_D_rear: float
