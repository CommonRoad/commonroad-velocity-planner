import numpy as np
from scipy.interpolate import CubicSpline


class SplineProfile:
    """
    Spline profile saving several values such as velocity, acceleration and jerk for interpolation at arc length.
    Note that this spline profile uses a downsampled reference path that only goes from initial state to goal state.
    """

    def __init__(
        self,
        path_length_per_point: np.ndarray,
        velocity_profile: np.ndarray,
        acceleration_profile: np.ndarray = None,
        goal_velocity: float = 0.01,
    ):
        """
        Spline profile saving several values such as velocity, acceleration and jerk for interpolation at arc length.
        :param path_length_per_point: arc length at point
        :param velocity_profile: velocity profile as np.ndarray
        :param acceleration_profile: acceleration profile as np.ndarray
        :param goal_velocity: default goal velocity.
        """
        self._path_length_per_point = path_length_per_point
        self._velocity_profile = velocity_profile
        self._velocity_spline = CubicSpline(
            self._path_length_per_point, self._velocity_profile
        )

        self._goal_velocity: float = goal_velocity

        interpoint_distance = path_length_per_point[1] - path_length_per_point[0]
        if acceleration_profile is None:
            acceleration_profile = np.zeros_like(velocity_profile)
            acceleration_profile[1:] = (
                # t = delta_v / s & a = delta_v / t --> delta_v2 / s = a
                np.power(np.diff(velocity_profile), 2)
                / interpoint_distance
            )
            acceleration_profile[0] = acceleration_profile[1]
        self._acceleration_profile = acceleration_profile

        self._acceleration_spline = CubicSpline(
            self._path_length_per_point, self._acceleration_profile
        )

        self._jerk_profile = (
            np.diff(self._acceleration_profile) / interpoint_distance
        ) * self._velocity_profile[:-1]

    @property
    def path_length_per_point(self) -> np.ndarray:
        """
        :return: (n,) interpoint arc length
        """
        return self._path_length_per_point

    @property
    def velocity_profile(self) -> np.ndarray:
        """
        :return: (n,) velocity profile per foint
        """
        return self._velocity_profile

    @property
    def velocity_spline(self) -> CubicSpline:
        """
        :return: velocity polyspline for interpolating between points
        """
        return self._velocity_spline

    @property
    def goal_velocity(self) -> float:
        """
        :return: goal velocity
        """
        return self._goal_velocity

    @property
    def acceleration_profile(self) -> np.ndarray:
        """
        :return: (n,) acceleration profile per point
        """
        return self._acceleration_profile

    @property
    def acceleration_spline(self) -> CubicSpline:
        """
        :return: (n,) acceleration spline
        """
        return self._acceleration_spline

    @property
    def jerk_profile(self) -> np.ndarray:
        """
        :return: (n,) jerk profile per point
        """
        return self._jerk_profile

    def interpolate_velocity_at_arc_lenth(
        self,
        s: [np.ndarray],
        clip_min: float = 0.0,
        goal_velocity: float = None,
    ) -> [np.ndarray]:
        """
        Interpoalte velocity at arc length
        :param s: arc
        :param clip_min: clip at minimum value
        :param goal_velocity: after the goal, this velocity is taken
        :return: interpolation velocity as np.ndarry
        """
        goal_vel: float = (
            goal_velocity if goal_velocity is not None else self._goal_velocity
        )

        # cubic splines have problems when exceeding last point, therefore just set it as last during interpolation
        interpolation = self._velocity_spline(s[s <= self._path_length_per_point[-1]])
        interpolation = np.concatenate(
            (
                interpolation,
                np.ones_like(s[s > self._path_length_per_point[-1]]) * goal_vel,
            )
        )

        # clip all values to lower bound
        interpolation = interpolation.clip(min=clip_min)

        return interpolation

    def interpolate_acceleration_at_arc_lenth(
        self, s: [np.ndarray], goal_acceleration: float = 0
    ) -> [np.ndarray]:
        """
        Interpoalte acceleration at arc length
        :param s: arc
        :param goal_acceleration: after the goal, this acceleration is taken
        :return: interpolation acceleration as np.ndarry
        """
        # cubic splines have problems when exceeding last point, therefore just set it as last during interpolation
        interpolation = self._acceleration_spline(
            s[s <= self._path_length_per_point[-1]]
        )
        interpolation = np.concatenate(
            (
                interpolation,
                np.ones_like(s[s > self._path_length_per_point[-1]])
                * goal_acceleration,
            )
        )

        return interpolation
