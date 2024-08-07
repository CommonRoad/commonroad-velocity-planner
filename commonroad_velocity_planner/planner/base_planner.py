from abc import ABC, abstractmethod

# own code case
from commonroad_velocity_planner.global_trajectory import GlobalTrajectory
from commonroad_velocity_planner.velocity_planning_problem import (
    VelocityPlanningProblem,
)
from commonroad_velocity_planner.configuration.velocity_planner_config import (
    VelocityPlannerConfig,
)

# typing
from typing import Optional


class BaseVelocityPlanner(ABC):
    """
    Base velocity planner class.
    """

    def __init__(self, velocity_planner_config: VelocityPlannerConfig) -> None:
        """
        :param velocity_planner_config: velocity planner config
        """
        self._config: VelocityPlannerConfig = velocity_planner_config
        self._v_max: float = None

    @property
    def config(self) -> VelocityPlannerConfig:
        """
        :return: velocity config
        """
        return self._config

    @property
    def v_max(self) -> float:
        """
        :return: maximum velocity
        """
        return self._v_max

    @abstractmethod
    def plan_velocity(
        self, problem: VelocityPlanningProblem, config: Optional[VelocityPlannerConfig]
    ) -> GlobalTrajectory:
        """
        plan velocity profile
        :param route: cr route object
        :return: cr global trajectory object
        """
        pass
