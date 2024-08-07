import enum
from enum import Enum

# own code base
from commonroad_velocity_planner.configuration.optimization_config import (
    OptimizationConfig,
    ConstraintType,
    VelMaxType,
    VelBoundType,
    JerkMinType,
    SolverBackend,
)
from commonroad_velocity_planner.configuration.vehicle_config import VehicleConfig
from commonroad_velocity_planner.configuration.velocity_planner_config import (
    VelocityPlannerConfig,
)


# typing
from typing import Optional


@enum.unique
class PlannerConfig(Enum):
    DEFAULT = 0


class ConfigurationBuilder:
    """
    Builds configuration
    """

    @staticmethod
    def get_predefined_configuration(
        planner_config: PlannerConfig = PlannerConfig.DEFAULT,
    ) -> VelocityPlannerConfig:
        """
        Builds predefined velocity planner config
        :param planner_config: planner config
        :return: velocity planner config object
        """
        if planner_config == PlannerConfig.DEFAULT:
            retconf: VelocityPlannerConfig = ConfigurationBuilder.build_default_config()

        else:
            raise NotImplementedError(f"The config {planner_config} is not implemented")

        return retconf

    @staticmethod
    def build_velocity_config(
        optimization_config: OptimizationConfig,
        vehicle_config: VehicleConfig,
        a_min: float,
        a_max: float,
        a_lateral_max: float,
        j_min: float,
        j_max: float,
        v_min_driving: float,
        v_max_street: float,
    ) -> VelocityPlannerConfig:
        """
        Build velocity planner config
        :param optimization_config: optimization config
        :param vehicle_config: vehicle configuration
        :param a_min: minimum acceleration
        :param a_max: maximum acceleration
        :param a_lateral_max: maximum lateral acceleration
        :param j_min: minimum jerk
        :param j_max: maximum jerk
        :param v_min_driving: minimum driving velocity
        :param v_max_street: maximum possible velocity if no rules
        :return:
        """
        return VelocityPlannerConfig(
            optimization_config=optimization_config,
            vehicle_config=vehicle_config,
            a_min=a_min,
            a_max=a_max,
            a_lateral_max=a_lateral_max,
            j_min=j_min,
            j_max=j_max,
            v_min_driving=v_min_driving,
            v_max_street=v_max_street,
        )

    @staticmethod
    def build_vehicle_config(
        mass: float,
        length_rear: float,
        length_front: float,
        inertia_z: float,
        tire_B_front: float,
        tire_C_front: float,
        tire_D_front: float,
        tire_B_rear: float,
        tire_C_rear: float,
        tire_D_rear: float,
    ) -> VehicleConfig:
        """
        Builds vehicle config
        :param mass: vehicle mass
        :param length_rear: length from cg to rear
        :param length_front: length from cg to front
        :param inertia_z: inertia z
        :param tire_B_front: pajecka B parameter for front tire
        :param tire_C_front: pajecka C parameter for front tire
        :param tire_D_front: pajecka D parameter for front tire
        :param tire_B_rear: pajecka B parameter for rear tire
        :param tire_C_rear: pajecka C parameter for rear tire
        :param tire_D_rear: pajecka D parameter for rear tire
        :return:
        """
        return VehicleConfig(
            mass=mass,
            length_rear=length_rear,
            length_front=length_front,
            inertia_z=inertia_z,
            tire_B_front=tire_B_front,
            tire_C_front=tire_C_front,
            tire_D_front=tire_D_front,
            tire_B_rear=tire_B_rear,
            tire_C_rear=tire_C_rear,
            tire_D_rear=tire_D_rear,
        )

    @staticmethod
    def build_optimization_config(
        velocity_maximization_type: VelMaxType,
        jerk_minimization_type: JerkMinType,
        jerk_min_weight: float,
        velocity_bound_type: VelBoundType,
        velocity_constraint: ConstraintType,
        velocity_over_weight: float,
        acceleration_constraint: ConstraintType,
        acceleration_over_weight: float,
        approximated_jerk_constraint: Optional[ConstraintType],
        approximated_jerk_over_weight: float,
        pseudo_jerk_constraint: Optional[ConstraintType],
        pseudo_jerk_over_weight: float,
        solver: SolverBackend,
    ) -> OptimizationConfig:
        """
        Builds optimization config
        :param velocity_maximization_type: type of velocity maximization
        :param jerk_minimization_type: type of jerk minimization
        :param jerk_min_weight: weight for jerk
        :param velocity_bound_type: type of velocity bound
        :param velocity_constraint: type constraint for velocity
        :param velocity_over_weight: weight for velocity in utility
        :param acceleration_constraint: type of acceleration constraint
        :param acceleration_over_weight: acceleration slack weight
        :param approximated_jerk_constraint: type for approximated jerk constraint
        :param approximated_jerk_over_weight: jerk slack weight
        :param pseudo_jerk_constraint: constraint type for pseudo jerk
        :param pseudo_jerk_over_weight: pseudo jerk weight
        :param solver: cvxpy solver
        :return: optimization config object
        """
        return OptimizationConfig(
            velocity_maximization_type=velocity_maximization_type,
            jerk_minimization_type=jerk_minimization_type,
            jerk_min_weight=jerk_min_weight,
            velocity_bound_type=velocity_bound_type,
            velocity_constraint=velocity_constraint,
            velocity_over_weight=velocity_over_weight,
            acceleration_constraint=acceleration_constraint,
            acceleration_over_weight=acceleration_over_weight,
            approximated_jerk_constraint=approximated_jerk_constraint,
            approximated_jerk_over_weight=approximated_jerk_over_weight,
            pseudo_jerk_constraint=pseudo_jerk_constraint,
            pseudo_jerk_over_weight=pseudo_jerk_over_weight,
            solver=solver,
        )

    @staticmethod
    def build_default_optimization_config() -> OptimizationConfig:
        """
        Builds default optimization config
        :return: optimization config
        """
        return ConfigurationBuilder.build_optimization_config(
            velocity_maximization_type=VelMaxType.SCALED_TO_APPROX_VEL,
            jerk_minimization_type=JerkMinType.APPROXIMATED_JERK,
            jerk_min_weight=30.0,
            velocity_bound_type=VelBoundType.APPROX_VEL_BOUNDED,
            velocity_constraint=ConstraintType.SOFT_QUADRATIC,
            velocity_over_weight=10e5,
            acceleration_constraint=ConstraintType.SOFT_QUADRATIC,
            acceleration_over_weight=6 * 10e6,
            approximated_jerk_constraint=None,
            approximated_jerk_over_weight=5 * 10e5,
            pseudo_jerk_constraint=ConstraintType.SOFT_QUADRATIC,
            pseudo_jerk_over_weight=5 * 10e5,
            solver=SolverBackend.CLARABEL,
        )

    @staticmethod
    def build_default_vehicle_config() -> VehicleConfig:
        """
        Builds default vehicle config
        :return: Vehicle Config
        """
        return ConfigurationBuilder.build_vehicle_config(
            length_rear=1.644,
            length_front=1.484,
            mass=2520,
            inertia_z=13600,
            tire_B_front=10,
            tire_C_front=1.3,
            tire_D_front=1.2,
            tire_B_rear=10,
            tire_C_rear=1.6,
            tire_D_rear=2.1,
        )

    @staticmethod
    def build_default_velocity_planner_config(
        optimization_config: OptimizationConfig, vehicle_config: VehicleConfig
    ) -> VelocityPlannerConfig:
        """
        Builds default velocity planner config
        :param optimization_config: optimization config
        :param vehicle_config: vehicle config
        :return: velocity planner config
        """
        return ConfigurationBuilder.build_velocity_config(
            optimization_config=optimization_config,
            vehicle_config=vehicle_config,
            a_lateral_max=2.0,
            a_min=-2.5,
            a_max=1.5,
            j_min=-4.0,
            j_max=3.6,
            v_min_driving=3.0,
            v_max_street=130.0 / 3.6,
        )

    @staticmethod
    def build_default_config() -> VelocityPlannerConfig:
        """
        Builds default config.
        For reasonable default values, confer
        https://www.researchgate.net/publication/326546961_Automated_Driving_System_ADS_High-Level_Quality_Requirements_Analysis_-_Driving_Behavior_Comfort#fullTextFileContent
        :return: default velocity planer config
        """
        optimization_config = ConfigurationBuilder.build_default_optimization_config()

        vehicle_config: VehicleConfig = (
            ConfigurationBuilder.build_default_vehicle_config()
        )

        return ConfigurationBuilder.build_default_velocity_planner_config(
            optimization_config=optimization_config, vehicle_config=vehicle_config
        )
