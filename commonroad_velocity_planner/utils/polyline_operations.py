import math

# third party
import numpy as np
from scipy.interpolate import CubicSpline

# commonroad
from commonroad_route_planner.utility.polyline_operations.polyline_operations import (
    compute_path_length_per_point,
    compute_length_of_polyline,
)

# typing
from typing import Optional, Tuple


def cubic_spline_arc_interpolation_2D(
    polyline: np.ndarray,
    start_distance: Optional[float] = None,
    stop_distance: Optional[float] = None,
    resampling_distance: float = 2,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Cubic spline interpolation from start to to stop value. If none given, take entire polyline
    :param polyline:
    :param start_distance:
    :param stop_distance:
    :return: resampled polyline, arc length per point, curvature per point
    """
    d_start = start_distance if start_distance is not None else 0
    d_inter = resampling_distance
    d_end = (
        stop_distance
        if stop_distance is not None
        else compute_length_of_polyline(polyline)
    )

    s: np.ndarray = compute_path_length_per_point(polyline)

    # may introduce rounding errors
    s_new = np.linspace(
        d_start,
        d_end,
        int(math.ceil((d_end - d_start) / d_inter)),
    )

    x_spline = CubicSpline(s, polyline[:, 0])
    y_spline = CubicSpline(s, polyline[:, 1])

    resampled_polyline = np.array(
        [
            x_spline(s_new),
            y_spline(s_new),
        ]
    ).T

    resampled_s: np.ndarray = compute_path_length_per_point(resampled_polyline)

    # counter rounding errors above
    max_idx = min(resampled_polyline.shape[0], s.shape[0])

    return resampled_polyline[:max_idx], resampled_s[:max_idx]
