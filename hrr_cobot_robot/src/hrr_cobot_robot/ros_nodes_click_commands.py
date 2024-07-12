# 3rd party import
import click

# ROS imports
import rospy
from geometry_msgs.msg import WrenchStamped
from hr_recycler_msgs.msg import ToolType
from std_srvs.srv import Trigger
from wsg_50_hw.grasping_controllers import ApplyCompliantGrasping

# hrr-cobot imports
from hrr_common import np2wrench_stamped, fix_prefix, get_param

# hrr-cobot-robot imports
from hrr_cobot_robot.hrr_cobot_handle import HrrCobotIf
from hrr_cobot_robot.hrr_cobot_control import HrrCobotControl
from hrr_cobot_robot.manipulation_skills import (
    CalibrationServer, Cutting, VacuumGraspingActionServer, ToolChangeActionServer,
    PneumaticFingerActionServer, SensitiveGrasping, Unscrew
)
from hrr_cobot_robot.tool_control import URDFHandles

__all__ = [
    "start_controller",
    "ft_sensor_calibrator",
    "compliant_cobot_DSA_control",
    "update_tool_URDF_server",
    "skill_server"
]


@click.command(
    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)
)
@click.option("--cobot-ns", default="/hrr_cobot", help="cobot namespace")
@click.option(
    "--activate-controller",
    type=click.Choice(
        ["sns-velocity", "sns-compliance", "joint-trajectory"], case_sensitive=False
    ),
    help="choose controller to activate",
    default="sns-velocity",
)
@click.option(
    "--sleep_for",
    default=1.5,
    help="Sleep time to react for controller manager updates in seconds",
)
@click.option("--nb-retries", type=int, default=5, help="number of retries")
def start_controller(
        activate_controller, cobot_ns, sleep_for, nb_retries
):
    """
    Simple controller manager helping tool to enable a sensor tracking based controller of choice.
    Sometimes the robot happens to be in undesired states, thus the robot state is reset via iterative loading and
    unloading of different controllers.

    If this script finally fails and returns an error. Please open the ``motion_handler.cod`` script on the robot panel,
    pause the ``STOP`` script and reset it manually on the TP5 panel.
    """
    cobot = HrrCobotIf.from_ros(cobot_prefix=fix_prefix(cobot_ns))
    cobot.deactivate_controllers()
    rospy.sleep(sleep_for)
    cobot.activate_controller(activate_controller)
    rospy.sleep(sleep_for)
    if cobot.needs_reset:
        rospy.logwarn(
            f"Robot needs reset before controller {activate_controller} can be used"
        )
        for _ in range(nb_retries):
            rospy.sleep(sleep_for)
            cobot.activate_controller(activate_controller)
            rospy.sleep(sleep_for)
            if not cobot.needs_reset:
                rospy.loginfo("robot is ready to receive data")
        rospy.logerr("Could not activate controller on robot")
    else:
        rospy.loginfo("robot is ready to receive data")
    rospy.loginfo(f"Robot State:\n{cobot}")


@click.command(
    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)
)
@click.option("--cobot-ns", default="/hrr_cobot", help="cobot namespace")
@click.option(
    "--loop-hz",
    default=170.0,
    help="update frequency for publishing the FT-sensor data",
)
@click.option(
    "--pub-B_F/--no-pub-B_F",
    default=True,
    help="publish calibrated FT data in base frame",
)
@click.option('--compile-numba/--no-compile-numba', default=False,
              help="start cobot and compile numba based kinematic handles")
def ft_sensor_calibrator(cobot_ns, loop_hz, pub_b_f, compile_numba):
    r"""
    FT-sensor calibration and publisher

    Since v0.1.4 this is implemented as an action-service as the calibration takes long

    Contains:

    * action-service for sensor-calibration
    * sensor track reference pose reset
    """
    r = rospy.Rate(loop_hz)
    cobot_ns = fix_prefix(cobot_ns)
    cobot = HrrCobotControl.from_ros(cobot_prefix=cobot_ns, compile_numba=compile_numba)
    action_server = CalibrationServer.from_ros(cobot_prefix=cobot_ns, cobot=cobot)
    action_server.cobot.deactivate_controllers()
    rospy.Service("~reset_sns_state", Trigger, action_server.cobot.reset_sns_service)
    b_ft_pub = (
        rospy.Publisher(f"{cobot_ns}B_F_ext", WrenchStamped, queue_size=10) if pub_b_f else None
    )
    while not rospy.is_shutdown():
        action_server.cobot.update_tf()
        if b_ft_pub is not None:
            b_ft_pub.publish(
                np2wrench_stamped(
                    action_server.cobot.B_F_ext, action_server.cobot.base_link
                )
            )
        r.sleep()


@click.command(
    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)
)
@click.option("--cobot-ns", default="/hrr_cobot", help="cobot namespace")
@click.option("--loop-hz", default=100.0, help="update frequency for publishing the FT-sensor data", )
@click.option("--pub-B_F/--no-pub-B_F", default=True, help="publish calibrated FT data in base frame", )
@click.option('--compile-nb/--no-compile-nb', default=False, help="flag tot compile numba based kinematic handles")
@click.option("--ad-grasp/--no-ad-grasp", default=True, help="spawn Action-server for adaptive-grasping", )
@click.option("--cut/--no-cut", default=True, help="spawn Action-server for cutting", )
@click.option("--fg-grasp/--no-fg-grasp", default=True, help="spawn Action-server for finger-grasping")
@click.option("--tool-chg/--no-tool-chg", default=True, help="spawn Action-server for tool-changing")
@click.option("--unscrew/--no-unscrew", default=True, help="spawn Action-server for unscrewing")
@click.option("--vc-grasp/--no-vc-grasp", default=True, help="spawn Action-server for vacuum-grasping")
def skill_server(cobot_ns, loop_hz, pub_b_f, compile_nb, ad_grasp, cut, fg_grasp, tool_chg, unscrew, vc_grasp, ):
    r"""
    Run skill-server as standalone node. This consists of

    1) setting up the cobot-handle, with optional compilation of inbuild numba-functions (~45 seconds)

    2) spawning action-services depending on set flags
       (adaptive-grasping, cutting, finger-grasping, tool-change, unscrewing, vacuum-grasp)

    3) F/T-sensor calibration routine and publishing of calibrated sensor-measurements

    """

    r = rospy.Rate(loop_hz)
    cobot_ns = fix_prefix(cobot_ns)
    cobot = HrrCobotControl.from_ros(cobot_prefix=cobot_ns, compile_numba=compile_nb)
    b_ft_pub = None
    skills = []
    if ad_grasp:
        rospy.loginfo(f"SKILL-SERVER -> initializing sensitive-grasping skill")
        skills.append(SensitiveGrasping.from_ros(cobot_prefix=cobot_ns, cobot=cobot,
                                                 skill_prefix=f"{cobot_ns}sensitive_grasping/"))
    if cut:
        rospy.loginfo(f"SKILL-SERVER -> initializing cutting skill")
        skills.append(Cutting.from_ros(cobot_prefix=cobot_ns, cobot=cobot))
    if fg_grasp:
        rospy.loginfo(f"SKILL-SERVER -> initializing finger-grasping skill")
        skills.append(PneumaticFingerActionServer.from_ros(cobot_prefix=cobot_ns, cobot=cobot,
                                                           skill_prefix=f"{cobot_ns}finger_grasping/"))
    if tool_chg:
        rospy.loginfo(f"SKILL-SERVER -> initializing tool-changing skill")
        skills.append(ToolChangeActionServer.from_ros(cobot_prefix=cobot_ns, cobot=cobot,
                                                      skill_prefix=f"{cobot_ns}tool_change_routine/"))
    if unscrew:
        rospy.loginfo(f"SKILL-SERVER -> initializing unscrewing skill")
        skills.append(Unscrew.from_ros(cobot_prefix=cobot_ns, cobot=cobot,
                                       skill_prefix=f"{cobot_ns}unscrewing/"))
    if vc_grasp:
        rospy.loginfo(f"SKILL-SERVER -> initializing vacuum-grasping skill")
        skills.append(VacuumGraspingActionServer.from_ros(cobot=cobot, cobot_prefix=cobot_ns,
                                                          skill_prefix=f"{cobot_ns}vacuum_grasping/"))
    if pub_b_f:
        b_ft_pub = rospy.Publisher(f"{cobot_ns}B_F_ext", WrenchStamped, queue_size=10)
    rospy.loginfo(f"SKILL-SERVER -> initializing FT-calibration server")
    calibration_prefix = f"{cobot_ns}{get_param(f'{cobot_ns}ft_sensor_ns', 'ft')}/"
    skills.append(CalibrationServer.from_ros(cobot_prefix=cobot_ns, cobot=cobot, calibration_prefix=calibration_prefix))
    ft_calib_idx = len(skills) - 1
    while not rospy.is_shutdown():
        try:
            skills[ft_calib_idx].cobot.update_tf()
            if pub_b_f:
                b_ft_pub.publish(np2wrench_stamped(skills[ft_calib_idx].cobot.B_F_ext,
                                                   skills[ft_calib_idx].cobot.base_link))
        except IndexError as e:
            rospy.logerr_once(f"SKILL-SERVER -> could not get cobot handle from calibration handle: {e}")
        except AttributeError as e:
            rospy.logerr_once(f"SKILL-SERVER -> could not publish B_F_ext: {e}")
        r.sleep()


@click.command(
    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)
)
@click.option("--cobot-ns", default="/hrr_cobot", help="cobot namespace")
@click.option("--gripper-ns", default="/hrr_cobot/gripper", help="gripper (WSG 50) namespace")
@click.option(
    "--loop-hz",
    default=30.0,
    help="update frequency for sending new velocity-commands to the robot",
)
def compliant_cobot_DSA_control(cobot_ns, gripper_ns, loop_hz):
    """
    This function allows to generate compliant robot behavior from
    WSG 50 DSA sensor - readings.
    It allows to generate either a force- or velocity based based
    compliant behavior and is best to be used for pHRI in free space.

    This function only subscribes to the actual control commands
    generated via each strategy separately and solely forwards
    the current control policy to the robot.

    """
    r = rospy.Rate(loop_hz)
    cobot = HrrCobotControl.from_ros(cobot_prefix=fix_prefix(cobot_ns))
    cobot.change_tool("wsg_50_dsa")
    apply_gripper_cmds = ApplyCompliantGrasping.from_ros(ros_param_ns=fix_prefix(gripper_ns))
    cobot.init_sns_vel()
    while not rospy.is_shutdown():
        if cobot.safety_check:
            apply_gripper_cmds.apply_via_cobot_handle(cobot)
            cobot.update()
        r.sleep()


@click.command(
    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)
)
@click.option("--cobot-ns", default="/hrr_cobot", help="cobot namespace")
@click.option("--topic-name", default="set_tool", help="ROS-topic to update robot URDF")
@click.option(
    "--loop-hz", default=1, help="update frequency for checking for an UPDATE"
)
@click.option(
    "--tool-parameter-name",
    default="tool_name",
    help="ROS-parameter name for current cobot tool",
)
@click.option('--robot-description', default="/hrr_cobot/robot_description",
              help="ROS-parameter name for URDF")
def update_tool_URDF_server(cobot_ns, topic_name, loop_hz, tool_parameter_name, robot_description):
    """This function allows to update the robot URDF after the tool has been changed."""

    r = rospy.Rate(loop_hz)
    cobot_ns = fix_prefix(cobot_ns)
    tool_parameter_name = f"{fix_prefix(cobot_ns)}{tool_parameter_name}"
    current_tool = get_param(tool_parameter_name, "nothing")
    urdf_handles = URDFHandles(cobot_ns=cobot_ns,
                               robot_name=rospy.get_param(f"{cobot_ns}robot_name", "hrr_cobot"),
                               tool_name=current_tool,
                               urdf_prefix=rospy.get_param(f"{cobot_ns}urdf_prefix", "hrr_cobot."))
    _ = rospy.Subscriber(topic_name, ToolType, urdf_handles.tool_cb, queue_size=100)

    while not rospy.is_shutdown():
        new_tool = get_param(tool_parameter_name, current_tool)
        if new_tool != current_tool:
            rospy.loginfo(f"URDF-updater SERVER -> ROS-parameter has changed -> update URDF to tool {new_tool}")
            urdf_handles.update_tool_name(tool_name=new_tool)
            current_tool = new_tool
        r.sleep()
