import numpy as np
from scipy.sparse import csc_matrix, identity

# third party
import osqp

# typing
from typing import Tuple


def smooth_path_elastic_bands(
    path: np.ndarray, max_deviation: float
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Elastic band smoothing algorithm
    :param path: path as position np array
    :param max_deviation: max deviation threshold for the points
    :return: returns x and y values separatly as two np.ndarrays
    """
    n = path.shape[0]
    q = np.zeros(2 * n)
    P = np.zeros((2 * n, 2 * n))

    for offset in [0, n]:
        P[offset, offset + 0] = 1
        P[offset, offset + 1] = -2
        P[offset, offset + 2] = 1
        P[offset + 1, offset + 0] = -2
        P[offset + 1, offset + 1] = 5
        P[offset + 1, offset + 2] = -4
        P[offset + 1, offset + 3] = 1
        P[offset + n - 1, offset + n - 1] = 1
        P[offset + n - 1, offset + n - 2] = -2
        P[offset + n - 1, offset + n - 3] = 1
        P[offset + n - 2, offset + n - 1] = -2
        P[offset + n - 2, offset + n - 2] = 5
        P[offset + n - 2, offset + n - 3] = -4
        P[offset + n - 2, offset + n - 4] = 1
    for k in range(2, n - 2):
        for offset in [0, n]:
            P[offset + k, offset + k - 2] = 1
            P[offset + k, offset + k - 1] = -4
            P[offset + k, offset + k] = 6
            P[offset + k, offset + k + 1] = -4
            P[offset + k, offset + k + 2] = 1

    A = identity(2 * n)
    lower_bound = np.concatenate((path[:, 0], path[:, 1]))
    upper_bound = lower_bound.copy()
    lower_bound -= max_deviation
    upper_bound += max_deviation

    solver = osqp.OSQP()
    solver.setup(
        P=csc_matrix(P),
        q=q,
        A=A,
        l=lower_bound,
        u=upper_bound,
        max_iter=20000,
        eps_rel=1.0e-4,
        eps_abs=1.0e-8,
        verbose=False,
    )
    res = solver.solve()

    return res.x[:n], res.x[n:]
