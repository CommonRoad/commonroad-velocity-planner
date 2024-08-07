import numpy as np
from scipy.interpolate import interp1d

# commonroad
from commonroad_route_planner.utility.polyline_operations.polyline_operations import (
    compute_path_length_per_point,
)

# typing
from typing import List, Tuple


def resample_polyline_adaptively(
    polyline: np.ndarray,
    path_length_per_point: np.ndarray,
    max_ds: float = 5.0,
    factor: float = 0.1,
    verbose=False,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Curtesy of Gerald Wuersching
    Adaptively resamples a given polyline according to the curvature.
    This function produces a polyline with non-uniform sampling distance. More samples are placed in parts with higher
    curvature.
    Tradeoff: number of waypoints vs. accuracy
    :param polyline: original polyline (equistantly sampled)
    :param path_length_per_point: arc length
    :param max_ds: maximum step for sampling
    :param factor: proportionality factor between arclength distance and curvature radius at a point
    :return: tuple (sampled_path, sampled_interpoint_distance)
    """

    raise NotImplementedError("Currently work in development")

    x: np.ndarray = polyline[:, 0]
    y: np.ndarray = polyline[:, 1]
    new_x_: List[float] = []
    new_y_: List[float] = []
    new_wp_: List[float] = []

    list_ds: List[float] = []

    # first point equals the first point of the original polyline
    new_x_.append(x[0])
    new_y_.append(y[0])
    new_wp_.append(path_length_per_point[0])

    # linear interpolation in x and y
    fx = interp1d(path_length_per_point, x)
    fy = interp1d(path_length_per_point, y)

    ds = max_ds
    for i in range(len(x)):
        # check distance between points and add it to list
        if ds < path_length_per_point[i] - new_wp_[-1] or i == (len(x) - 1):
            if i == (len(x) - 1):
                # if last point is reached, just add last point of original polyline
                ds = path_length_per_point[-1] - new_wp_[-1]
            s = new_wp_[-1] + ds
            if verbose:
                print(ds)
            list_ds.append(ds)

            # interpolate for new x, y, curvature at pathlength s
            new_x_.append(float(fx(s)))
            new_y_.append(float(fy(s)))
            new_wp_.append(s)
            # reset
            ds = max_ds

        # update current step size given curvature
        curvature_radius = 1 / abs(path_length_per_point[i])
        ds = min(ds, 1 / factor * curvature_radius)
        ds = max(ds, path_length_per_point)

    resampled_polyline: np.ndarray = np.column_stack((new_x_, new_y_))
    resampled_path_length_per_point: np.ndarray = compute_path_length_per_point(
        resampled_polyline
    )

    return resampled_polyline, resampled_path_length_per_point
