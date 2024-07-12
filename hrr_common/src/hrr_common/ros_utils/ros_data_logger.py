#!/usr/bin/env python3
"""
Data logger and helper
------------------------

Utterly unimportant but useful.
This file allows to record data and generate pandas dataframes and plots without a huge programming overhead
"""
from abc import ABC, abstractmethod
import sys
import warnings

import numpy as np
import pandas as pd
import spatialmath as sm

__all__ = ["PosData", "PoseData", "ArrayData", "init_error_dict", "init_data_dict", "log_helper"]

if float(sys.version[:3]) <= 3.5:
    warnings.warn("you are running 2, parts of these data logger expect python-spatialmath data, "
                  "which requires python 3.6+")


def fuse_tex_str(s1, s2, seperator="_"):
    """fuse two strings, where ``s1`` may be given as a LaTeX-string, i.e. in the form of $xx$
    """
    if "$" in s1:
        tmp = s1.split('$')
        return f"${{{tmp[1]}}}{seperator}{{{s2}}}$"
    return f"${{{s1}}}{seperator}{{{s2}}}$"


class BaseLog(ABC):
    """ Default logging utility class
    """

    def __init__(self, prefix):
        self._prefix = prefix

    def _df_key(self, elem):
        # type: (object) -> str
        return fuse_tex_str(self._prefix, elem)

    @property
    @abstractmethod
    def data(self):
        """data
        return the content of the Logging handle as a dictionary

        Returns:
            dict: collected data samples indexed by name
        """

    def as_df(self):
        r"""as_df
        transform buffer to `pandas Dataframe <https://pandas.pydata.org/pandas-docs/stable/reference/api/pandas.DataFrame.html>`_
        for eased post-processing or plotting (:py:meth:`~plot`)

        Returns:
            pd.DataFrame: data dictionary as a pandas Dataframe
        """
        return pd.DataFrame(self.data)

    def plot(self, **kwargs):
        """plot collected data using the `pandas plot function <https://pandas.pydata.org/pandas-docs/stable/reference/api/pandas.DataFrame.plot.html#pandas.DataFrame.plot>`_.
        """
        self.as_df().plot(**kwargs)


class PosData(BaseLog):
    """
    Simple Wrapper to store xyz data in a buffer

    Buffer resolved as list for xyz data.

    usage online where ``x`` is data we obtain from e.g. a ros-subscriber

    .. code-block:: python

        X = PosData('$\mathbf{x}$')
        r = rospy.Rate(100)
        for _ in trange(2000):
            X.append(x)
            r.sleep()
        X.plot()

    will plot the data using the plot utility of a pandas Dataframe
    """

    def __init__(self, prefix):
        super(PosData, self).__init__(prefix)
        self.x = []
        self.y = []
        self.z = []

    def append(self, x):
        try:
            self.x.append(x[0])
            self.y.append(x[1])
            self.z.append(x[2])
        except:
            pass

    @property
    def data(self):
        """data [summary]

        Returns:
            dict: [description]
        """
        return {self._df_key(elem): np.array(getattr(self, elem)) for elem in ("x", "y", "z")}

    def reset(self):
        """reset, i.e. empty buffers
        """
        self.x[:] = []
        self.y[:] = []
        self.z[:] = []


class PoseData(BaseLog):
    """Similar to :py:class:`~PosData` allows to store for 6DoF Cartesian recordings
    """

    def __init__(self, prefix):
        super(PoseData, self).__init__(prefix)
        self.pos = PosData(fuse_tex_str(prefix, r"\mathrm{pos}", seperator="^"))
        self.rot = PosData(fuse_tex_str(prefix, r"\mathrm{rot}", seperator="^"))

    def append(self, x):
        try:
            self.pos.append(x[:3])
            self.rot.append(x[3:])
        except:
            pass

    @property
    def data(self):
        """data combined data from pos and rotation

        Returns:
            dict: key->value based buffer data
        """
        return {**self.pos.data, **self.rot.data}

    def reset(self):
        """reset, i.e. empty buffers
        """
        self.pos.reset()
        self.rot.reset()


class ArrayData(BaseLog):
    """ArrayData

    Allows to log arbitrary vector data to be logged.
    Similar to :py:class:`~PosData`, but stores arbitrary vector combinations in individual lists
    name indexing is by default introduced by enumeration (see :py:meth:`~reset` and :py:meth:`~append`)

    """

    def __init__(self, dof, prefix):
        super(ArrayData, self).__init__(prefix)
        self._dof = dof
        self.reset()

    def append(self, x):
        r"""append new data ``x``, provided as an Iterable such as numpy array, lists, tuples etc.


        Raises:
            KeyError: if :math:`|x|`, where :math:`n` is ``self._dof``, i.e. DoF

        Args:
            x (Iterable): new vector to be added to buffer.
        """
        if x.shape == (self._dof,):
            for i, elem in enumerate(x):
                getattr(self, f"elem_{i}").append(elem)

    @property
    def data(self):
        return {self._df_key(i + 1): np.array(getattr(self, f"elem_{i}")) for i in range(self._dof)}

    def reset(self):
        """reset all buffers to an empty list. enumerate all """
        for i in range(self._dof):
            setattr(self, f"elem_{i}", [])


def log_helper(data, value, prefix, pos="{pos}", rot="{rot}"):
    r"""
    Simple logging helper

    Args:
        data(dict):  logging dict containing multiple key-value entries, where values ar of type :py:class:`~PosData`
        value(np.ndarray): 6-DoF Cartesian value
        prefix(str):  base string
        pos(str, optional): translation key, default to '{pos}'
        rot(str, optional): rotation key, default to '{rot}'

    Returns:

    """
    data[f"{prefix}_{pos}"].append(value[0:3])
    data[f"{prefix}_{rot}"].append(value[3:6])


def init_error_dict(data=None):
    """Init error dict for logging experimental data"""
    if data is None:
        data = dict()
    data[r"\epsilon_{pos}"] = PosData(r"{\epsilon}_{pos}")
    data[r"\epsilon_{rot}"] = PosData(r"{\epsilon}_{rot}")
    data[r"\dot{\epsilon}_{pos}"] = PosData(r"\dot{\epsilon}_{pos}")
    data[r"\dot{\epsilon}_{rot}"] = PosData(r"\dot{\epsilon}_{rot}")
    data[r"\Delta_{pos}"] = PosData(r"\Delta_{pos}")
    data[r"\Delta_{rot}"] = PosData(r"\Delta_{rot}")
    data[r"\epsilon_{f}"] = PosData(r"\epsilon_{f}")
    data[r"\epsilon_{\tau}"] = PosData(r"\epsilon_{\tau}")
    return data


def init_data_dict():
    data = init_error_dict()
    data["f_raw"] = PosData('f')
    data["f_msr"] = PosData('f')
    data["q"] = ArrayData(6, r'{\bf{q}^{\bf{R}}}')
    data["p_is"] = PosData(r'{\bf{x}^{\bf{R}}}')
    data["v_x"] = PosData(r'\tilde{\bf{x}}')
    data["v_xt"] = PosData(r'\tilde{\dot{\bf{x}}}')
    data["dT"] = PosData(r'\Delta_{T}')
    data["contacts"] = []
    return data
