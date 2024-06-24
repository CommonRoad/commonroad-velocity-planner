# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_route_planner.route import Route

# own code base
from commonroad_velocity_planner.config.velocity_config import VelocityConfig



def create_velocity_config_from_route_and_planning_problem(
        planning_problem: PlanningProblem,
        route: Route,
        v_max: float,
        a_lateral_max: float,
        a_min: float,
        a_max: float,
        j_min: float,
        j_max: float,
        v_terminal: float,
        a_terminal: float,
) -> VelocityConfig:
    """
    Factory method to create a config dataclass from a route and a planning problem
    :param planning_problem: cr planning problem
    :param route: cr route from route planner
    :param v_terminal: stopping velocity
    :param a_terminal: stopping acceleration
    :param v_max: maximum velocity
    :param a_lateral_max: maximum lateral acceleration
    :param a_min: maximum breaking acceleration
    :param a_max: maximum acceleration
    :param j_min: minimal jerk
    :param j_max: maximal jerk
    :return: velocity config dataclass
    """

    return VelocityConfig(
        reference_path=route.reference_path,
        path_curvature=route.path_curvature,
        distances=route.interpoint_distances,
        num_steps=route.reference_path.shape[0],
        v_initial=planning_problem.initial_state.velocity,
        v_terminal=v_terminal,
        a_initial=planning_problem.initial_state.acceleration,
        a_terminal=a_terminal,
        v_max=v_max,
        a_min=a_min,
        a_max=a_max,
        a_lateral_max=a_lateral_max,
        j_min=j_min,
        j_max=j_max,
    )
