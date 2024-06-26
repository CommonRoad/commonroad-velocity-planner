import math
import time
from logging import Logger, DEBUG

import numpy as np
import csv
from scipy.spatial import KDTree

# commonroad
from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_route_planner.route_generator import RouteGenerator
from commonroad_route_planner.route import Route
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.scenario.state import InitialState, CustomState
from commonroad.prediction.prediction import Trajectory, TrajectoryPrediction
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import ObstacleType, DynamicObstacle
from commonroad.common.file_writer import OverwriteExistingFile


# own code base
from commonroad_velocity_planner.race_line import RaceLine, raceline_factory_from_route
from commonroad_velocity_planner.util.parking_vehicles_heuristic import heuristically_identify_if_vehicle_on_street_is_parking


# typing
from typing import List




class VelocityBehaviorPlanner:
    """
    Limits the velocity profile given the scenario to a reasonable value.
    """

    _logger = Logger(__name__)
    _logger.setLevel(DEBUG)

    def __init__(
            self,
            vehicle_length: float,
            vehicle_width: float,
    ) -> None:
        # vehicle params
        self._vehicle_width = vehicle_width
        self._vehicle_length = vehicle_length




    def update_velocity(
            self,
            scenario: Scenario,
            reference_path: np.ndarray,
            ref_path_length_per_point: np.ndarray,
            velocity_profile: np.ndarray,
            vehicle_position_x: float,
            vehicle_position_y: float,
            vehicle_velocity_x: float,
            vehicle_velocity_y: float,
            current_speed_limit: float = 10,
            lateral_threshold: float = 0.5,
            parking_pass_speed_ms: float = 1.5,
            look_ahead_sec: float = 10
    ) -> np.ndarray:
        """
        Updates velocity profile given parking vehicles
        :param scenario:
        :param race_line:
        :param vehicle_position_x:
        :param vehicle_position_y:
        :param vehicle_velocity_x:
        :param vehicle_velocity_y:
        :param current_speed_limit:
        :param lateral_threshold:
        :param parking_pass_speed_ms:
        :param look_ahead_sec:
        :return: velocity profile
        """

        # look ahead in m
        look_ahead_m: float = look_ahead_sec * min(
            math.sqrt(vehicle_velocity_x**2 + vehicle_velocity_y**2), current_speed_limit
        )

        # hard cut on maximum currently detected speed limit
        velocity_profile = velocity_profile.clip(max=current_speed_limit)

        # for all parking vehicles in radius, check the distance left for driving of ego vehicle. If this distance
        # is below threshold, reduce speed
        distance_list: List[float] = [np.inf]
        for dyn_obs in scenario.dynamic_obstacles:
            if(
                    math.sqrt(
                        (dyn_obs.initial_state.position[0] - vehicle_position_x)**2
                        + (dyn_obs.initial_state.position[1] - vehicle_position_y)**2)
                    > look_ahead_m
            ):
                continue

            self._logger.debug("Starting Heuristic Check")

            parking, distance_obs = heuristically_identify_if_vehicle_on_street_is_parking(
                dyn_obstacle=dyn_obs,
                lanelet_network=scenario.lanelet_network,
            )
            if(parking):
                self._logger.debug(f"Heuristic Found parking vehicle with id {dyn_obs.obstacle_id}")
                distance_list.append(distance_obs)

        # clipp to parking pass distance
        smallest_distance_parking: float = min(distance_list)
        print(len(distance_list))
        if(smallest_distance_parking < lateral_threshold):
            self._logger.debug("identified vehicles and clip")

            kd_tree = KDTree(reference_path)
            _, idx_start = kd_tree.query(np.asarray([vehicle_position_x, vehicle_position_y]))
            # FIXME smarter conversion
            idx_end: int = ref_path_length_per_point[idx_start:].tolist().index(
                min(ref_path_length_per_point[idx_start:].tolist(),
                    key=lambda x: abs(x - look_ahead_m - ref_path_length_per_point[idx_start])
                    )
            ) + idx_start


            self._logger.debug(f"idx_start={idx_start}  --  idx_end={idx_end}")
            self._logger.debug(
                f"start_ref_path={ref_path_length_per_point[idx_start]} "
                f"-- end_ref_path={ref_path_length_per_point[idx_end]}"
                f"-- horizon_m={look_ahead_m}")


            velocity_profile[idx_start:idx_end] = velocity_profile[idx_start:idx_end].clip(max=parking_pass_speed_ms)


        return velocity_profile






if __name__ == "__main__":
    scenario_xml: str = "/home/tmasc/projects/velocity_planner/commonroad-velocity-planner/scenarios/DEU_GarchingCampus2D-2.xml"

    scenario, planning_problem_set = CommonRoadFileReader(
        scenario_xml
    ).open()
    # retrieve the first planning problem in the problem set
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    initial_state = InitialState(
        position=np.asarray([698076, 5349079]),
        orientation=0,
        velocity=0,
        acceleration=0,
        yaw_rate=0,
        slip_angle=0,
        time_step=0
    )

    custom_state = CustomState(
        position=np.asarray([698076, 5349079]),
        orientation=0,
        velocity=0,
        acceleration=0,
        yaw_rate=0,
        slip_angle=0,
        time_step=1
    )
    cr_trajectory = Trajectory(initial_state.time_step, [initial_state, custom_state])
    shape = Rectangle(width=2, length=4)

    trajectory_prediction = TrajectoryPrediction(
        trajectory=cr_trajectory,
        shape=shape
    )
    # obstacle generation
    dynamic_obstacle = DynamicObstacle(
        obstacle_id=300000,
        obstacle_type=ObstacleType.CAR,
        obstacle_shape=shape,
        initial_state=initial_state,
        prediction=trajectory_prediction,
    )

    scenario.add_objects(dynamic_obstacle)

    fw = CommonRoadFileWriter(scenario, planning_problem_set)
    fw.write_to_file(
        "/home/tmasc/projects/velocity_planner/commonroad-velocity-planner/scenarios/DEU_GarchingCampus2D-3.xml",
        OverwriteExistingFile.ALWAYS
    )

    route_planner = RoutePlanner(
        lanelet_network=scenario.lanelet_network,
        planning_problem=planning_problem,
        extended_search=False,
    )
    # plan routes, and save the routes in a route candidate holder
    route_generator: "RouteGenerator" = route_planner.plan_routes()

    route: "Route" = route_generator.retrieve_shortest_route()

    velocity_profile: np.ndarray = np.asarray(
        [11.22624101179257, 11.226241011812036, 11.207447614044767, 11.17088478133093, 11.11757478482141,
         11.048536094533366, 10.964779358570004, 10.86705426510856, 10.755768027474156, 10.631236823381935,
         10.493801018651736, 10.343837602090378, 10.18177514645936, 10.008112069315963, 9.823439172236808,
         9.628467698487206, 9.424064476788963, 9.211296117421037, 8.99148468979867, 8.765881287576686, 8.53427432529704,
         8.296194114329976, 8.051070435787711, 7.79823930879745, 7.536924769359638, 7.266211126130552,
         6.985005052010269, 6.691983246678748, 6.3855184180296725, 6.063571487939716, 5.723529029996721,
         5.361947562655207, 4.974130501835453, 4.553397766895087, 4.0901922308426615, 3.587437946558144,
         3.0839250768740296, 2.682754204795505, 2.4238213984363313, 2.316395114201407, 2.3350062015951645,
         2.434794027649343, 2.5775902364685805, 2.7398275698592993, 2.912077610555835, 3.0802068145475956,
         3.2225012701472657, 3.3329744729877047, 3.4143972586531857, 3.4706346503668537, 3.504884027029302,
         3.5197717119585334, 3.5174717756072944, 3.4998065895878323, 3.4674518292678087, 3.4195314241089694,
         3.3544336383551503, 3.270475782005048, 3.1660952965688427, 3.0404140887635878, 2.894962982543353,
         2.739993973782543, 2.6078111186848463, 2.499013673285632, 2.4226522898025604, 2.3996837655769707,
         2.46065763656925, 2.637637521323317, 2.9531286884047074, 3.4018344457367036, 3.880179894519412,
         4.342700524938045, 4.776089854182652, 5.178024626532406, 5.551094071676094, 5.900605160365569,
         6.230379703350559, 6.541509035405709, 6.834791867064892, 7.111149127120339, 7.371523170908864,
         7.616828525404536, 7.847929169954055, 8.065629636435785, 8.270673284724504, 8.463744212841076,
         8.645470892459523, 8.816430497609415, 8.977153374350802, 9.128127364887444, 9.269801847423627,
         9.402591435067652, 9.5268793218815, 9.643020287815334, 9.751343385864965, 9.852154339586408, 9.945737680123234,
         10.032358650907492, 10.112264906199659, 10.18568802722135, 10.252844877132251, 10.313938813687395,
         10.369160776169888, 10.418690261165091, 10.462696199932125, 10.50133774853101, 10.534765000461334,
         10.563119630343639, 10.586535476106826, 10.605139066215317, 10.619050097660482, 10.62838186973636,
         10.633241678005616, 10.633731172325344, 10.629946682333642, 10.6219563781102, 10.60978220055028,
         10.59339725473082, 10.572728461680235, 10.547681852657602, 10.51815870700613, 10.484055289871614,
         10.445262568652765, 10.401665906730008, 10.353144732972618, 10.299572185509815, 10.24081472828677,
         10.176731739033947, 10.107175067491102, 10.031988563090524, 9.951007571882728, 9.864058403372018,
         9.770957769247664, 9.671512197930635, 9.56551743166654, 9.45275781695497, 9.333005704954012, 9.206020886920095,
         9.071550101889299, 8.929326671378604, 8.779070341428023, 8.620487449652268, 8.453271589974694,
         8.277105029404758, 8.091661253702291, 7.896609204392086, 7.691620054246845, 7.476377810581022,
         7.250595732453675, 7.014041662391386, 6.766577184810444, 6.508218517502949, 6.2392320758181095,
         5.960286249389667, 5.6726958378351, 5.378821681983985, 5.082733722019545, 4.791324439823978, 4.516189031637775,
         4.276116078416564, 4.0977804313891495, 3.9813987753358098, 3.92004007972657, 3.903585702841167,
         3.9212770500851644, 3.962766663252373, 4.018537995662346, 4.08055033731867, 4.143061348640073,
         4.201941285476113, 4.2541775581077514, 4.297696942308665, 4.331476123848238, 4.3553882326045485,
         4.37016426518095, 4.377338263515374, 4.379280455500476, 4.377999496482251, 4.374243302959278,
         4.3662722942019645, 4.353325466231618, 4.335711028191553, 4.314811808769789, 4.292603875359562,
         4.2717094285382045, 4.256062170837427, 4.251346559868192, 4.250265786068324, 4.246400117966482,
         4.234451011449918, 4.21018489925665, 4.170666121935724, 4.114862092011088, 4.043437016718028,
         3.957704275872363, 3.858924839307524, 3.7476772603001973, 3.6235025215220507, 3.485614836689771,
         3.3339601275315833, 3.1702939060020996, 3.0009904306766675, 2.8444089167723163, 2.7399432231299845,
         2.704397250820998, 2.706361657783431, 2.719424159065375, 2.727499494395989, 2.7250670676623012,
         2.716993019432339, 2.7183364486197195, 2.7541376965250706, 2.857063265660533, 2.977774002944776,
         3.084843580442655, 3.1621896619044283, 3.205430924614936, 3.217472984099767, 3.2017597689155015,
         3.160778546385741, 3.0962665446044335, 3.0093243359393007, 2.8990025961495, 2.761295908481492,
         2.5896363485289062, 2.3745143249165395, 2.1009129536065085, 1.7416688255346406, 1.2331186488852126,
         0.0006941070351495798]
    )

    vbp: VelocityBehaviorPlanner = VelocityBehaviorPlanner(
        vehicle_width=2.5,
        vehicle_length=4.5
    )


    print(velocity_profile.shape[0])
    print(route.reference_path.shape[0])

    upsample_coefficient: int = int(route.reference_path.shape[0] / velocity_profile.shape[0])

    vel_profile_adj = velocity_profile.repeat(upsample_coefficient, axis=0)

    vel_profile_adj = np.append(vel_profile_adj, [vel_profile_adj[-1], vel_profile_adj[-1], vel_profile_adj[-1], vel_profile_adj[-1], vel_profile_adj[-1], vel_profile_adj[-1], vel_profile_adj[-1], vel_profile_adj[-1], vel_profile_adj[-1], vel_profile_adj[-1]], axis=0)

    t_start = time.perf_counter()
    vel_profile_updated = vbp.update_velocity(
        scenario=scenario,
        reference_path=route.reference_path,
        ref_path_length_per_point=route.path_length_per_point,
        velocity_profile=vel_profile_adj,
        vehicle_position_x=698064,
        vehicle_position_y=5349082,
        vehicle_velocity_x=5,
        vehicle_velocity_y=0,
        parking_pass_speed_ms=2,
        lateral_threshold=5,
    )

    print(f"Time of execution: {time.perf_counter() - t_start}")

    with open('file.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(vel_profile_adj)
        writer.writerow(vel_profile_updated)



    print(vel_profile_adj.shape[0])
    print(route.reference_path.shape[0])














