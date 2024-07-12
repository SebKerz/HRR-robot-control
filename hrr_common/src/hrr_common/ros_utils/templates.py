"""
Base Classes & Templates
---------------------------

In order to unify instantiation over ROS-parameters, this module contains
a collection of templates that are used throughout this module.

Thus, each class that shall later be used as handle, should at least inherit from
:py:class:`~RosBaseHandle`.
If the handle in addition intends to communicate with the controller-manager, it should
inherit from :py:class:`~BaseController`.


"""
from abc import ABC, abstractmethod

import rospy

import hrr_common.ros_utils.controller_manager as hrr_cm
import hrr_common.ros_utils.helper_handles as hrr_hlp

__all__ = ["RosBaseHandle", "BaseController"]


class RosBaseHandle(ABC):
    """
    This is a basic ros handle that unifies the initialization
    within ROS-packages / nodes, such that we initialize classes
    solely from ROS-parameters via

    .. code-block::
        python

        new_ctrl = RosBaseHandle.from_ros()

    see :py:meth:`~from_ros` for further insights
    """

    @abstractmethod
    def init_ros(self, *_, **__):
        pass

    @classmethod
    @abstractmethod
    def _from_ros(cls, cobot_prefix, **kwargs):
        pass

    @classmethod
    def from_ros(cls, cobot_prefix, **kwargs):
        r"""Failsafe initialization method.
        Proposed sequence is:

        call custom class-method ``_from_ros`` where all class attributes are
        set from ROS-parameters as needed.
        Afterwards, call ``init_ros`` to initialize ROS-IF, i.e. subscribers / publishers / etc.

        The motivation here is that you can then either use your class or by manual construction.
        Let's assume the handle below

        .. code-block::
            python

            class MyHandle(RosBaseHandle):
                # ...
                def init_ros(self, my_topic):
                    self._my_pub = rospy.Publisher(my_topic, TopicClass, queue_size=1)

                @classmethod
                def _from_ros(cls):
                    out = cls(rospy.get_param("some_parameter_I_need_in__init__"))
                    out.init_ros(rospy.get_param("my_topic_name"))
                    return out

        then, we can either create an new instance solely from ROS:

        .. code-block::
            python

            my_handle = MyHandle.from_ros()

        or by hand

        .. code-block::
            python

            parameter_I_need_in_init = 12
            my_handle = MyHandle(parameter_I_need_in_init)
            my_handle.init_ros("/the_amazing_ROS_topic")

        Args:
            cobot_prefix(str, optional): prefix for loading ROS-parameters. Defaults to "~", i.e. local namespace
        """
        try:
            return cls._from_ros(cobot_prefix=hrr_hlp.fix_prefix(cobot_prefix), **kwargs)
        except KeyError as e:
            rospy.logwarn(f"could not initialize handle due @ {rospy.get_name()} to missing ROS-parameter {e}")


class BaseController(RosBaseHandle, ABC):
    """
    This class extends the :py:class:`~RosBaseHandle`.
    Controller base class that handles communication with the controller manager

    - (de)activation
    - failsafe setting of controller name

    It also implements the ``init_ros`` function to set the controller-manager namespace.

    The main benefit is given as

    * the controller is loaded upon initialization (see :py:meth:`~controller_name`)
    * the communication with the controller manager is simplified

    one can directly check the controller status via

    .. code-block::
        python

        c = BaseController.from_ros()
        c.active
        Out[1]: False

        c.activate()
        c.active
        Out[2]: True

        c.deactivate()
        c.active
        Out[2]: False

    and also wrap the helper function :py:func:`hrr_common.ros_utils.controller_manager.print_all_controllers`
    via

    .. code-block::
        python

        c.list_controllers()
    """
    
    controller_manager_ns = ""

    def __init__(self) -> None:
        super().__init__()
        self._controller_name = ""
        self._cobot_prefix = ""

    def init_ros(self, cobot_prefix, *_, **__):
        self._cobot_prefix = cobot_prefix
        if len(self.controller_manager_ns) > 0:
            self.controller_manager_ns = rospy.get_param(f"{cobot_prefix}controller_manager_ns", self.controller_manager_ns)
        self.controller_manager_ns = rospy.get_param(f"{cobot_prefix}controller_manager_ns")

    @property
    def controller_name(self):
        assert self._controller_name is not None, "your code is messed up"
        return self._controller_name

    @controller_name.setter
    def controller_name(self, value) -> None:
        """
        Set controller name of class.
        This also checks the controller to be known by the controller manager
        Then the controller is loaded to allow instant (de)activation and connect
        to the dedicated ROS-IF, i.e. (action-)services / topics.

        Args:
            value(str): controller name as known to the controller manager
        """
        def get_name_from_list(ctrl_list):
            return [x[0] for x in ctrl_list]
        assert value in get_name_from_list(hrr_cm.get_all_controllers(self.controller_manager_ns)), \
            f"controller {value} is unknown for controller-manager {self.controller_manager_ns}"
        if value not in get_name_from_list(hrr_cm.get_controller_names(self.controller_manager_ns)):
            hrr_cm.load_controller(self.controller_manager_ns, value)
        self._controller_name = value

    @property
    def controller_ns(self):
        return hrr_hlp.fix_prefix(f"{self._cobot_prefix}{self.controller_name}")

    @property
    def active(self) -> bool:
        return self._controller_name in hrr_cm.active_controllers(self.controller_manager_ns)

    def activate(self):
        if not self.active:
            hrr_cm.activate_controller(self.controller_manager_ns, self._controller_name)

    def deactivate(self):
        if self.active:
            hrr_cm.deactivate_controller(self.controller_manager_ns, self._controller_name)

    def reload(self):
        hrr_cm.reload_controller(self.controller_manager_ns, self._controller_name, check=True)

    def list_controllers(self):
        hrr_cm.print_all_controllers(self.controller_manager_ns)

