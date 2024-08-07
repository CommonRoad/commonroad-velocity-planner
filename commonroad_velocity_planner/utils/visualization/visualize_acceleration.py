import numpy as np
import matplotlib.pyplot as plt

from commonroad_velocity_planner.spline_profile import SplineProfile


# typing
from typing import List


def visualize_acceleration_over_arclength(
    path_length_per_point: np.ndarray,
    velocity_profile: SplineProfile,
    save_path: str,
    save_img: bool = False,
    saving_format: str = "png",
    test: bool = False,
) -> None:
    """
    Visualization for the velocity
    :param path_length_per_point: arc length per point
    :param velocity_profile: spline velocity provile
    :param save_path: save image to
    :param save_img: if true, saves image, otherwise displays it
    :param saving_format: saving format
    :param test: if test, image is neither shown nor displayed
    """
    plt.figure()

    x: List[float] = []
    y: List[float] = []
    v_last = velocity_profile.interpolate_velocity_at_arc_lenth(
        np.asarray([path_length_per_point[0]])
    )
    s_last = path_length_per_point[0]
    for idx in range(1, path_length_per_point.shape[0]):

        v_current = velocity_profile.interpolate_velocity_at_arc_lenth(
            np.asarray([path_length_per_point[idx]])
        )
        s_current = path_length_per_point[idx]

        delta_t = abs((s_current - s_last) / ((v_current - v_last) / 2 + v_last))

        a = (v_current - v_last) / delta_t

        v_last = v_current
        s_last = s_current

        x.append(path_length_per_point[idx])
        y.append(a)

    plt.plot(x, y, "r")
    plt.title("Acceleration over arc length")

    # save or show scenario
    if not test:
        if save_img:
            plt.savefig(save_path, format=saving_format, dpi=300, bbox_inches="tight")
        else:
            plt.show()
