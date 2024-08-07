import enum
import logging

# commonroad
from commonroad_route_planner.route import Route

# own code base
from commonroad_velocity_planner.spline_profile import SplineProfile
from commonroad_velocity_planner.planner.lp_optimization_planner import (
    LinearProgramPlanner,
)
from commonroad_velocity_planner.configuration.velocity_planner_config import (
    VelocityPlannerConfig,
)
from commonroad_velocity_planner.global_trajectory import (
    GlobalTrajectory,
    factory_from_route_and_velocity_profile,
)
from commonroad_velocity_planner.velocity_planning_problem import (
    VelocityPlanningProblem,
)
from commonroad_velocity_planner.planner.bang_bang_planner import BangBangSTPlanner


# typing
from typing import Tuple, Union


@enum.unique
class ImplementedPlanners(enum.Enum):
    LinearProgramPlanner = 0
    BangBangSTPlanner = 1


class IVelocityPlanner:
    """
    Velocity planner interface.
    """

    def __init__(self):
        """
        Interface for all velocity planners.
        """
        self._logger = logging.Logger(name="IVelocityPlanner", level=logging.WARNING)

        # TODO: Currently leave them as members so people have an easier time debugging
        self._route: Route = None
        self._velocity_planner_config: VelocityPlannerConfig = None
        self._vpp: VelocityPlanningProblem = None
        self._planner: Union[LinearProgramPlanner, BangBangSTPlanner] = None
        self._spline_profile: SplineProfile = None
        self._global_trajectory: GlobalTrajectory = None
        self._velocity_planner: ImplementedPlanners = None

    @property
    def route(self) -> Route:
        """
        :return: cr route object
        """
        return self._route

    @property
    def velocity_planning_problem(self) -> VelocityPlanningProblem:
        """
        :return: current velocity planning problem
        """
        return self._vpp

    @property
    def config(self) -> VelocityPlannerConfig:
        """
        :return: velocity planner config
        """
        return self._velocity_planner_config

    @property
    def planner(self) -> Union[LinearProgramPlanner]:
        """
        :return: solver interface
        """
        return self._planner

    def plan_velocity(
        self,
        route: Route,
        velocity_planning_problem: VelocityPlanningProblem,
        planner_config: VelocityPlannerConfig,
        velocity_planner: ImplementedPlanners = ImplementedPlanners.LinearProgramPlanner,
        return_spline_profile: bool = False,
    ) -> Union[GlobalTrajectory, Tuple[GlobalTrajectory, SplineProfile]]:
        """
        Plans velocity profile and returns a global trajectory object, or a Tuple [Global Trajectory, Spline Profile].
        :param route: cr route
        :param planning_problem: cr planning problem
        :param planner_config: velocity planner config
        :param velocity_planner: selected velocity planner
        :param resampling_distance: resampling distance to speed up profile
        :param default_goal_velocity: default goal velocity is none is given
        :param return_spline_profile: if true, returns a Tuple with global_trajectory and spline_velocity_profile
        :return: returns either only the global trajectory or a Tuple with global_trajectory and spline_velocity_profile
        """

        # TODO: For now make them members as this is usually easier to debug
        self._velocity_planner: ImplementedPlanners = velocity_planner
        self._velocity_planner_config: VelocityPlannerConfig = planner_config
        self._route: Route = route
        self._vpp: VelocityPlanningProblem = velocity_planning_problem

        # init planner
        self._init_planner()

        # plan and create spline profile
        self._spline_profile: SplineProfile = self._planner.plan_velocity(
            problem=self._vpp, config=planner_config
        )

        # create global trajectory
        self._global_trajectory: GlobalTrajectory = (
            factory_from_route_and_velocity_profile(
                route=route,
                velocity_profile=self._spline_profile,
                velocity_planning_problem=self._vpp,
            )
        )

        if return_spline_profile:
            retval = (self._global_trajectory, self._spline_profile)
        else:
            retval = self._global_trajectory

        return retval

    def _init_planner(self) -> None:
        """
        Init planner given api choice
        """
        if self._velocity_planner == ImplementedPlanners.LinearProgramPlanner:
            self._planner = LinearProgramPlanner(config=self._velocity_planner_config)
            self._logger.info(f"Initialized planner: {self._velocity_planner}")
        elif self._velocity_planner == ImplementedPlanners.BangBangSTPlanner:
            self._planner = BangBangSTPlanner(config=self._velocity_planner_config)
            self._logger.info(f"Initialized planner:  {self._velocity_planner}")
        else:
            self._logger.error(f"{self._velocity_planner} is not implemented")
            NotImplementedError(f"{self._velocity_planner} is not implemented")
