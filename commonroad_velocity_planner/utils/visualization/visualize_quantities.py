import numpy as np
import matplotlib.pyplot as plt


def visualize_velocity_over_arclength(
    path_length_per_point: np.ndarray,
    velocity_profile: np.ndarray,
    v_min: float,
    v_max: float,
    save_path: str,
    save_img: bool = False,
    saving_format: str = "png",
    test: bool = False,
) -> None:
    """
    Visualizes velocity over arc length
    :param path_length_per_point: arc length per point
    :param velocity_profile: velocity profile
    :param v_min: minimum velocity threshold
    :param v_max: maximum velocity threshold
    :param save_path: path to same image to
    :param save_img: if true, saves image, otherwise displays it
    :param saving_format: saving format
    :param test: if test, neither displays nor saves images
    """
    plt.cla()
    plt.clf()

    plt.title("Velocity over arc length")

    plt.plot(path_length_per_point, np.ones_like(velocity_profile) * v_min, "black")
    plt.plot(path_length_per_point, np.ones_like(velocity_profile) * v_max, "black")
    plt.plot(path_length_per_point, velocity_profile)

    # save or show scenario
    if not test:
        if save_img:
            plt.savefig(save_path, format=saving_format, dpi=300, bbox_inches="tight")
        else:
            plt.show()


def visualize_acceleration_over_arclength(
    path_length_per_point: np.ndarray,
    acceleration_profile: np.ndarray,
    a_min: float,
    a_max: float,
    save_path: str,
    save_img: bool = False,
    saving_format: str = "png",
    test: bool = False,
) -> None:
    """
    Visualizes acceleration over arc length
    :param path_length_per_point: arc length per point
    :param acceleration_profile: acceleration profile
    :param a_min: minimum acceleration threshold
    :param a_max: maximum acceleration threshold
    :param save_path: path to save image to
    :param save_img: if true, saves image, otherwise displays it
    :param saving_format: saving format
    :param test: if true, neither saves nor displays figures
    """
    plt.figure()

    plt.title("Acceleration over arc length")

    plt.plot(path_length_per_point, np.ones_like(acceleration_profile) * a_min, "black")
    plt.plot(path_length_per_point, np.ones_like(acceleration_profile) * a_max, "black")

    plt.plot(path_length_per_point, acceleration_profile)

    # save or show scenario
    if not test:
        if save_img:
            plt.savefig(save_path, format=saving_format, dpi=300, bbox_inches="tight")
        else:
            plt.show()
