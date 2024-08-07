import copy

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import rgb2hex, Normalize
from matplotlib import cm
from mpl_toolkits.axes_grid1 import make_axes_locatable

# commonroad
from commonroad.geometry.shape import Circle
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.scenario import Scenario

# own code base
from commonroad_velocity_planner.global_trajectory import GlobalTrajectory
from commonroad_velocity_planner.velocity_planning_problem import (
    VelocityPlanningProblem,
)

# typing
from typing import List, Tuple

# cmap for coloring the velocity profile
cmap = cm.get_cmap("plasma")


def visualize_global_trajectory(
    scenario: Scenario,
    velocity_planning_problem: VelocityPlanningProblem,
    global_trajectory: GlobalTrajectory,
    save_path: str,
    size_x: float = 10.0,
    save_img: bool = False,
    saving_format: str = "png",
    test: bool = False,
) -> None:
    """
    Visualizes global trajectory in scenario
    :param scenario: cr scenario
    :param velocity_planning_problem: velocity planning problem object
    :param global_trajectory: global trajectory object
    :param save_path: path to save image to
    :param size_x: fig size
    :param save_img: if true, saves image, otherwise displays it
    :param saving_format: saving format
    :param test: if true, neither displays nor saves iamge
    """
    plt.cla()
    _ = plt.figure(figsize=(20, 10))

    # get plot limits from reference path
    plot_limits: List[float] = obtain_plot_limits_from_reference_path(
        global_trajectory.reference_path, margin=20
    )
    ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])

    renderer = MPRenderer(plot_limits=plot_limits, figsize=(size_x, size_x / ratio_x_y))
    renderer.draw_params.dynamic_obstacle.draw_icon = True

    scenario.draw(renderer)

    scenario.lanelet_network.draw(renderer)
    velocity_planning_problem.planning_problem.draw(renderer)

    v_min, v_max = get_velocity_min_max_from_trajectory(
        global_trajectory.velocity_profile
    )
    for idx in range(global_trajectory.reference_path.shape[0]):
        draw_route_state(
            renderer,
            reference_point=global_trajectory.reference_path[idx],
            velocity=global_trajectory.velocity_profile[idx],
            v_min=v_min,
            v_max=v_max,
        )

    # draw scenario and renderer
    renderer.render()
    # plt.axis('off')
    v_start = global_trajectory.velocity_profile[
        global_trajectory.planning_problem_start_idx
    ]
    v_goal = global_trajectory.velocity_profile[
        global_trajectory.planning_problem_goal_idx
    ]
    plt.title(
        f"v_min={round(v_min, 2)}  --  v_max={round(v_max, 2)} | v_start={round(v_start, 2)}  --  v_goal={round(v_goal, 2)}"
    )

    # colorbar work-around
    divider = make_axes_locatable(plt.gca())
    cax = divider.append_axes("right", size="5%", pad=0.05)
    norm = Normalize(vmin=v_min, vmax=v_max)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    plt.colorbar(sm, cax=cax, cmap="plasma", orientation="vertical")

    # save or show scenario
    if not test:
        if save_img:
            plt.savefig(save_path, format=saving_format, dpi=300, bbox_inches="tight")
            print(f"saving image to {save_path}")
        else:
            plt.show()


def draw_route_state(
    renderer: MPRenderer,
    reference_point: np.ndarray,
    velocity: float,
    v_min: float,
    v_max: float,
    point_radius: float = 0.1,
) -> None:
    """
    Draws route state
    :param renderer: cr MPRenderer
    :param reference_point: point to draw as (2,) np.ndarry
    :param velocity: velocity of point
    :param v_min: v_min
    :param v_max: v_max
    :param point_radius: radius to display point
    """

    normalized_velocity: float = (
        (velocity - v_min) / (v_max - v_min) if not np.isclose(v_max, v_min) else 0
    )
    rbg_color = cmap(normalized_velocity)
    hex_color = rgb2hex(rbg_color)
    draw_params = copy.copy(renderer.draw_params)
    draw_params.shape.facecolor = hex_color
    draw_params.shape.edgecolor = hex_color

    occ_pos = Circle(radius=point_radius, center=reference_point)
    occ_pos.draw(renderer, draw_params=draw_params)


def get_velocity_min_max_from_trajectory(
    velocity_profile: np.ndarray,
) -> Tuple[float, float]:
    """
    Gets min and max velocity from global trajectory for color coding.
    :param velocity_profile: velocity profile as (n,2) np.ndarray
    :return: tuple[v_min, v_max]
    """
    min_velocity: float = np.min(velocity_profile)
    max_velocity: float = np.max(velocity_profile)
    return (min_velocity, max_velocity)


def obtain_plot_limits_from_reference_path(
    reference_path: np.ndarray, margin: float = 10.0
) -> List[int]:
    """
    Obtrains plot limits from reference path
    :param reference_path: reference path (2,) np.ndarray
    :return: list [xmin, xmax, ymin, xmax] of plot limits
    """
    x_min = min(reference_path[:, 0])
    x_max = max(reference_path[:, 0])
    y_min = min(reference_path[:, 1])
    y_max = max(reference_path[:, 1])

    plot_limits = [x_min - margin, x_max + margin, y_min - margin, y_max + margin]
    return plot_limits
