from abc import ABC, abstractmethod
import enum

import numpy as np
from scipy.interpolate import CubicSpline

# own code base
from commonroad_velocity_planner.preprocessing.elastic_band_algo import (
    smooth_path_elastic_bands,
)


@enum.unique
class SmoothingStrategy(enum.Enum):
    NONE = 1
    ELASTIC_BAND = 2


class ICurvatureSmoother:
    """
    Curvature smoother interface
    """

    @staticmethod
    def smooth(
        smoothing_strategy: SmoothingStrategy,
        reference_path: np.ndarray,
        path_length_per_point: np.ndarray,
    ) -> np.ndarray:
        """
        Smooth reference path given the strategy
        :param smoothing_strategy: smoothing strategby
        :param path_curvature: curvature of the path per point
        :param reference_path: reference path
        :param path_length_per_point: arc length per point
        :param interpoint_distance: distance between points
        :return: smoothed reference path
        """

        if smoothing_strategy == SmoothingStrategy.NONE:
            smoother = NoCurvatureSmoothingStrategy()
        elif smoothing_strategy == SmoothingStrategy.ELASTIC_BAND:
            smoother = ElasticBandCurvatureSmoothingStrategy()
        else:
            raise NotImplementedError(
                f"Smoothing strategy: {smoothing_strategy} not implemented"
            )

        return smoother.smooth(
            reference_path=reference_path, path_length_per_point=path_length_per_point
        )


class BaseCurvatureSmoothingStrategy(ABC):
    """
    Abstract base class for Curvature smoother
    """

    @abstractmethod
    def smooth(
        self,
        reference_path: np.ndarray,
        path_length_per_point: np.ndarray,
    ) -> np.ndarray:
        """
        Smoothes the curvature of the reference path.
        :param reference_path: reference path as np array
        :param path_length_per_point: path length at each point

        :return: smoothed curvature
        """
        pass


class NoCurvatureSmoothingStrategy(BaseCurvatureSmoothingStrategy):
    """
    No Curvature smoothing
    """

    def smooth(
        self,
        reference_path: np.ndarray,
        path_length_per_point: np.ndarray,
    ) -> np.ndarray:
        """
        Smooth reference path given the strategy
        :param reference_path: reference path
        :param path_length_per_point: arc length per point
        :return: smoothed reference path
        """
        return reference_path


class ElasticBandCurvatureSmoothingStrategy(BaseCurvatureSmoothingStrategy):
    """
    Elastic Band Curvature Smoothing
    """

    def __init__(self, max_deviation: float = 0.15) -> None:
        """
        :param max_deviation: Max deviation allowed.
        """
        self._max_deviation: float = max_deviation

    @property
    def max_deviation(self) -> float:
        """
        :return: Maximum deviation for the points
        """
        return self._max_deviation

    def smooth(
        self,
        reference_path: np.ndarray,
        path_length_per_point: np.ndarray,
    ) -> np.ndarray:
        """
        Smooth reference path with elastic band algorithm.
        :param reference_path: reference path
        :param path_length_per_point: arc length per point
        :return: smoothed reference path
        """

        x, y = smooth_path_elastic_bands(reference_path, self._max_deviation)
        dists = np.sqrt((x[:-1] - x[1:]) ** 2 + (y[:-1] - y[1:]) ** 2)
        s = np.zeros(reference_path.shape[0])
        s[1:] = np.cumsum(dists)

        x_spline = CubicSpline(s, x)
        y_spline = CubicSpline(s, y)
        x = x_spline(path_length_per_point)
        y = y_spline(path_length_per_point)

        return np.array([x, y]).T
