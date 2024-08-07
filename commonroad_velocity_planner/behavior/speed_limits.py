import numpy as np

# commonroad
from commonroad.scenario.lanelet import LaneletNetwork


def get_speed_limits_from_lanelet_network(
    reference_path: np.ndarray,
    lanelet_network: LaneletNetwork,
    max_speed_limit: float = 130 / 3.6,
) -> np.ndarray:
    """
    Calculated speed limits from road network
    :param reference_path: reference path as np.ndarray
    :param lanelet_network: cr lanelet network object
    :param max_speed_limit: maximum speed limit in m/s
    :return: speed limit per point of reference path
    """
    res = np.full(reference_path.shape[0], max_speed_limit)
    for i, p in enumerate(reference_path):
        lanelet_ids = lanelet_network.find_lanelet_by_position([p])[0]
        if len(lanelet_ids) > 0:
            for l_id in lanelet_ids:
                lanelet = lanelet_network.find_lanelet_by_id(l_id)
                for ts_id in lanelet.traffic_signs:
                    ts = lanelet_network.find_traffic_sign_by_id(ts_id)
                    for ts_el in ts.traffic_sign_elements:
                        if ts_el.traffic_sign_element_id.name == "MAX_SPEED":
                            try:
                                speed_limit = float(ts_el.additional_values[0])
                                res[i] = min(res[i], speed_limit)
                            except ValueError:
                                pass
    return res
