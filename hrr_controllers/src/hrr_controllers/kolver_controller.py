"""
KOLVER Pluto 10 ROS-Python handler
-----------------------------------

This controller interface uses a digital encoder for enabling
the KOLVER screwdriver program and modes.

One 3-bit address encoder is used to set the program of the KOLVER EDU 2AE/TOP box,
which are connected to the digital OUT-pins of the R1C control unit of the
COMAU-robot.

The controller stacks two :py:class:`~hrr_controllers.dout_encoder_controller.EncoderController`
instances that control the 

* KOLVER program
* KOLVER enabling / rotation direction

ROS-parameters
^^^^^^^^^^^^^^^^^^^^




"""
import rospy
from hrr_common.ros_utils.helper_handles import get_param
from hrr_controllers.dout_encoder_controller import EncoderController

__all__ = ["ScrewdriverControl"]


class KolverProgramController(EncoderController):

    def __init__(self):
        super(KolverProgramController, self).__init__(frame_id="screwdriver")

    @classmethod
    def _from_ros(cls, cobot_prefix, **__):
        out = cls()
        out.init_ros(controller_name=get_param(f"{cobot_prefix}kolver_program_controller_name"), cobot_prefix=cobot_prefix, **__)
        return out


class KolverCommandController(EncoderController):

    def __init__(self):
        super(KolverCommandController, self).__init__(frame_id="screwdriver")

    @classmethod
    def _from_ros(cls, cobot_prefix, **__):
        out = cls()
        out.init_ros(controller_name=get_param(f"{cobot_prefix}kolver_command_controller_name"), cobot_prefix=cobot_prefix, **__)
        return out


class ScrewdriverControl:

    def __init__(self):
        self.program_handle = KolverProgramController()
        self.command_handle = KolverCommandController()
        self._start_mode = ""

    @classmethod
    def from_ros(cls, cobot_prefix, **__):
        out = cls()
        out.program_handle = KolverProgramController.from_ros(cobot_prefix=cobot_prefix)
        out.command_handle = KolverCommandController.from_ros(cobot_prefix=cobot_prefix)
        out._start_mode = get_param(f"{out.command_handle.controller_ns}start_mode", "loosening")
        out.command_handle.timeout = get_param(f"{out.command_handle.controller_ns}runtime", out.command_handle.timeout)
        return out

    def activate(self):
        """activate program handle and run control handle.
        
        If program handle is not 0, send current program to controller via
        :py:meth:`~hrr_controllers.dout_encoder_controller.EncoderController.mode_id`
        """
        self.program_handle.activate()
        if self.program_handle.mode_id > 0:
            self.program_handle.mode_id = self.program_handle.mode_id
        self.command_handle.activate()

    def deactivate(self):
        """deactivate both program and run control handle"""
        self.command_handle.deactivate()
        self.program_handle.deactivate()

    def run_program(self, program, dt=2.0, sleep_time=0.1):
        """Run a specific program on a KOLVER for a predefined time
        
        Note:
            As the KOLVER control box has a delay during changing the program,
            the ``sleep_time`` needs to be sufficiently high to allow for 
            program switching and enabling of the screwdriver

        Args:
            program (Union[str, int]): program number or name 
            dt (float, optional): runtime for current program. Defaults to 2.0.
            sleep_time (float, optional): sleep time between program change and enabling. Defaults to 0.1.

        Raises:
            ValueError: if program is neither a program name nor integer
        """
        if isinstance(program, str):
            self.program_handle.mode_name = program
        elif isinstance(program, int):
            self.program_handle.mode_id = program
        else:
            raise ValueError("program need to be a string or integer")
        self.command_handle.timeout = dt
        rospy.sleep(sleep_time)
        self.command_handle.mode_name = self._start_mode

    @property
    def timeout(self):
        """
        class handle to ease setting runtime for underlying classes via function or
        :py:meth:`~run_program`.
        
        Args:
            float: runtime in seconds

        Returns:
            float: current runtime
        
        Raises:
            AssertionError: If a value smaller than 0 is set.
        """
        return self.command_handle.timeout

    @timeout.setter
    def timeout(self, value):
        assert value >= 0.0, "timeout needs to be non-negative"
        self.command_handle.timeout = value
        self.command_handle.mode_id = self.command_handle.mode_id


if __name__ == "__main__":
    rospy.init_node("test_kolver_controller")
    Kolver = ScrewdriverControl.from_ros(cobot_prefix="/hrr_cobot")
    Kolver.activate()
    rospy.sleep(1.0)
    Kolver.run_program(4, 2.0)
    Kolver.deactivate()
