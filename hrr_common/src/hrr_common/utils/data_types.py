#!/usr/bin/env python
"""
Data files and format
----------------------

In order to ease data recording / saving, this file contains a collection of pythonized data types.

Note:
    for python versions 3.7-, the module `dataclasses <https://docs.python.org/3/library/dataclasses.html>`_
    is not available.
"""

import spatialmath as sm
import numpy as np

__all__ = ["CalibData", "CalibDataFull", "DataBuffer"]

try:
    from dataclasses import dataclass


    @dataclass
    class CalibData:
        wrench: np.ndarray  # (Nx6)
        R: np.ndarray  # (Nx3x3)


    @dataclass
    class CalibDataFull(CalibData):
        p: np.ndarray  # (Nx3)
        q: np.ndarray  # (Nx6)

except (ImportError, ModuleNotFoundError):
    from collections import namedtuple

    CalibData = namedtuple("CalibData", ("wrench", "R"))
    CalibDataFull = namedtuple("CalibDataFull", ("wrench", "R", "p", "q"))


class DataBuffer:

    def __init__(self):
        self._wrench = []
        self._R = []
        self._p = []
        self._q = []

    def reset(self):
        self._wrench.clear()
        self._R.clear()
        self._p.clear()
        self._q.clear()

    def add(self, wrench, T_b_e, q):
        # type: (np.ndarray, sm.SE3, np.ndarray) -> None
        self._wrench.append(wrench)
        self._p.append(T_b_e.t)
        self._R.append(T_b_e.R)
        self._q.append(q)

    @property
    def data(self):
        # type: () -> CalibData
        return CalibData(np.r_[self._wrench], np.r_[self._p])

    @property
    def all_data(self):
        # type: () -> CalibDataFull
        return CalibDataFull(wrench=np.r_[self._wrench], p=np.r_[self._p], R=np.r_[self._R], q=np.r_[self._q])
