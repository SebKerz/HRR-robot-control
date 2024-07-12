#!/usr/bin/env python3
"""
HRR Cobot Observer
---------------------

A useful data instance that can be used to track data os needed.
Intended Usage

.. code-block::
    python

    from hrr_cobot_robot import *

    cobot = HrrCobotControl
    obs = HrrCobotObserver(100, 'q')

    for t in range(5000):
        cobot.update()
        obs.update(cobot)


"""
# 3rd party imports
import numpy as np
import quaternion
from typing import Union

# ROS imports
import rospy

# hrr-cobot-robot imports
from hrr_cobot_robot.hrr_cobot_handle import HrrCobotIf
from hrr_cobot_robot.hrr_cobot_control import HrrCobotControl

_Cobot = Union[HrrCobotIf, HrrCobotControl]

__all__ = ["HrrCobotObserver", "GripperObserver"]


class HrrCobotObserver:

    def __init__(self, buf_size, *attributes):

        self._N = int(buf_size)
        self._attributes = attributes
        self._needs_init = True

    def set_buffers(self, cobot):
        def shape_helper(x):
            try:
                if isinstance(x, quaternion.quaternion):
                    return 1
                return x.shape
            except (AttributeError, IndexError):
                return 1

        self._attributes = list(filter(lambda x: hasattr(cobot, x), self._attributes))
        for a in self._attributes:
            try:
                tmp = getattr(cobot, a)
                shape = shape_helper(tmp)
                if shape != 1:
                    setattr(self, a, np.zeros([self._N, ] + list(shape), dtype=tmp.dtype))
                else:
                    try:
                        setattr(self, a, np.zeros(self._N, dtype=tmp.dtype))
                    except AttributeError:
                        setattr(self, a, np.zeros(self._N))
            except AttributeError as e:
                rospy.logerr(f"could not set buffer for {a} due to {e}")
            except TypeError as e:
                rospy.logerr(f"could not set buffer for {a} due to {e}")
        self._needs_init = False

    @staticmethod
    def _roll_buf(buf, new_elem):
        buf = np.roll(buf, -1, 0)
        buf[-1] = new_elem.copy()
        return buf

    def update(self, cobot) -> None:
        """
        Update all buffers via rolling buffer command

        # shift buffer by 1 to left
        # add new element to end of buffer

        Args:
           cobot(_Cobot): current cobot handle      cobot:
        """

        if self._needs_init:
            self.set_buffers(cobot)

        for x in self._attributes:
            setattr(self, x, self._roll_buf(getattr(self, x), getattr(cobot, x)))

    def drop(self, cobot, *args, **kwargs) -> dict:
        """
        Drop current data as a dictionary that can be saved to file, DB, etc.

        Allows arbitrary member attributes to be added to dictionary

        * args   -> get member attributes from cobot
        * kwargs -> external key-value data

        Args:
            cobot(_Cobot): current cobot handle

        Returns:
            dict: data dict for current class instance
        """
        if self._needs_init:
            print("\033[01m\033[31m !!current observer is not initialized!!\033[0m")
            return dict()

        def get_data(k):
            try:
                return getattr(cobot, k).copy()
            except (AttributeError, TypeError):
                try:
                    return getattr(cobot, k)
                except AttributeError:
                    print(f"skip unknown attribute {k}")

        members = {k: get_data(k) for k in args}
        for k in self._attributes:
            v = getattr(self, k)
            try:
                members[k] = v[v[:, 0] != 0.0]
            except IndexError:
                if v.dtype == quaternion.quaternion:
                    members[k] = v[np.where(v != np.quaternion(0, 0, 0, 0))]
                else:
                    members[k] = v
        for k, v in kwargs.items():
            if v is not None:
                try:
                    members[k] = v.copy()
                except AttributeError:
                    members[k] = v
        return members

    def save(self, save_path, cobot, *args, **kwargs) -> None:
        """
        Simple wrapper around :py:meth:`drop` to save data directly in file ``save_path``.

        Args:
            save_path (Path): path to filename
            cobot(_Cobot): current cobot handle
        """
        assert (save_path.parent.exists())
        np.save(save_path, self.drop(cobot, *args, **kwargs), allow_pickle=True)

    def reset(self) -> None:
        """Set all buffer entries to 0"""
        if self._needs_init:
            return
        for x in self._attributes:
            getattr(self, x).fill(0.0)

    @property
    def N(self):
        return self._N


class GripperObserver(HrrCobotObserver):

    def __init__(self, *args):
        super(GripperObserver, self).__init__(*args)
        self._align_buf = None

    def set_buffers(self, cobot):
        super().set_buffers(cobot)
        self._align_buf = np.zeros((self.N, 6))

    def update(self, cobot) -> None:
        super(GripperObserver, self).update(cobot)
        try:
            self._align_buf = self._roll_buf(self._align_buf, cobot.gripper.alignment_error)
        except AttributeError as e:
            rospy.logerr_once(f"could not store alignment error: {e}")
            pass

    def drop(self, *args, sort_by="q", **kwargs) -> dict:
        out = super(GripperObserver, self).drop(*args, **kwargs)
        if len(out) > 0:
            out["align_buf"] = self._align_buf[np.linalg.norm(getattr(self, sort_by), axis=1) != 0.0, :]
        return out

    def reset(self) -> None:
        super(GripperObserver, self).reset()
        self._align_buf.fill(0.0)
