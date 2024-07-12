import numpy as np

import rospy

from .ipy_utils import in_ipynb

__all__ = ["check_unit_vectors", "get_range", "rate_dt_helper", "normalize_vec"]

def check_unit_vectors(v1, v2):
    """check_unit_vectors
    simple helper function to check if two vectors are either a zero-vector or unit-vector.
    If both-vectors are unit-vectors, they further need to be perpendicular, i.e.

    .. math::

        \bf{v1}^{\top} \bf{v2} = \bf{0}

    Args:
        v1 (np.ndarray): vector to be checked
        v2 (np.ndarray): vector to be checked

    Returns:
        bool: True, if check valid
    """
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    return any([(n1 == 0 and n2 == 0), (n1 != 0 and n2 == 0), (n1 == 0 and n2 != 0),
                (n1 != 0 and n2 != 0 and np.dot(v1, v2) == 0)])


def rate_dt_helper(r=None, dt=None):
    """Get sleep function and current time rate
    As the rate and sleep commands are different this function wraps the dedicated sleep command depending
    on the input data provided

    Args:
        r (Union[rospy.Rate, None], optional): rate of current node. Defaults to None.
        dt (Union[float, None], optional): temporal update step size. Defaults to None.

    Returns:
        Tuple[float, callable]: Δt and sleep handle
    """
    def sleep():
        rospy.sleep(dt)
    assert not(dt is None and r is None), "either provide dt "
    if r is None:
        assert dt > 0, "time step Δt <= 0"
        return dt, sleep
    else:
        return r.sleep_dur.to_sec(), r.sleep


def get_range(N, visualize=True):
    """get_range function

    allows to set range as needed. Switches to to trange if required and checks against ipython-support.

    Args:
        N (int): range size
        visualize (bool, optional): enable trange. Defaults to True.

    Returns:
        range / trange
    """
    if visualize:
        if in_ipynb():
            from tqdm.notebook import trange
        else:
            from tqdm import trange
        return trange(N)
    else:
        return range(N)


def normalize_vec(normal):
    """simple vector normalizer helper function

    Args:
        normal (np.ndarray): (normal) vector

    Returns:
        np.ndarray: unit vector of

    Raises:
        AssertionError: if vector has zero-length
    """
    nn = np.linalg.norm(normal)
    if nn == 0.0:
        return np.zeros(normal.shape)
    return normal / nn

