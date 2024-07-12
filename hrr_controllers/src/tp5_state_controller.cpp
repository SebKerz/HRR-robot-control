//
// Created by gabler on 26.01.21.
//

#include <hrr_controllers/tp5_state_controller.h>
#include <pluginlib/class_list_macros.h>

namespace hrr_controllers
{
  comau_msgs::CartesianPose arrayToMsg(const std::array<double, 6> &rpy_pose_tool)
  {
    comau_msgs::CartesianPose pose{};
    pose.x = rpy_pose_tool[0];
    pose.y = rpy_pose_tool[1];
    pose.z = rpy_pose_tool[2];
    pose.roll = rpy_pose_tool[3];
    pose.pitch = rpy_pose_tool[4];
    pose.yaw = rpy_pose_tool[5];
    return pose;
  }

  struct TP5RosParams
  {
    std::string robot_hw_name{"comau_driver"};
    std::string topic_name{"comau_cartesian_data"};    
    double publish_rate{};
    bool valid{true};

    TP5RosParams(const ros::NodeHandle &nh)
    {
      valid = getParam<double>(nh, TP5StateController::m_log_name, "publish_rate", publish_rate);
      if (valid)
          valid = getParam<std::string>(nh, TP5StateController::m_log_name, "robot_hw_name", robot_hw_name);  
      nh.param<std::string>("robot_status_topic_name", topic_name, topic_name);
      nh.param<std::string>("robot_hw_name", robot_hw_name, robot_hw_name);
    }

    [[nodiscard]] std::string print_params() const
    {
      std::stringstream ss;
      ss << "\n\tpublish_rate:  \t" << publish_rate 
         << "\n\ttopic_name:    \t" << topic_name 
         << "\n\trobot_hw_name: \t" << robot_hw_name;
      return ss.str();
    }
  };

  bool TP5StateController::init(hardware_interface::comau::TP5StateInterface *hw, ros::NodeHandle &nh)
  {
    TP5RosParams params{nh};
    m_publish_rate = params.publish_rate;
    try
    {
      ROS_DEBUG("[%s] Connecting to robot-hw %s", m_log_name.c_str(), params.robot_hw_name.c_str());
      m_hw_handle = hw->getHandle(params.robot_hw_name);
      m_comau_robot_status_pub =
          std::make_unique<realtime_tools::RealtimePublisher<comau_msgs::TP5State>>(nh, params.topic_name, 100);
      ROS_DEBUG("[%s] Connected to hardware %s\n Added publisher: %s", m_log_name.c_str(), params.robot_hw_name.c_str(),
                params.topic_name.c_str());
      return true;
    }
    catch (...)
    {
      ROS_ERROR("[%s] Failed to connect to robot-hw %s", m_log_name.c_str(), params.robot_hw_name.c_str());
      return false;
    }
  }

  void TP5StateController::update(const ros::Time &time, const ros::Duration &period)
  {
    if (m_publish_rate > 0.0 && m_last_publish_time + ros::Duration(1.0 / m_publish_rate) < time)
    {
      auto [err, status] = comau_driver::statusToMsg(m_hw_handle.getStatus().status);
      if (err)
      {
        status = "ERROR: " + status;
      }
      std::vector<double> joints{m_hw_handle.getJointValues()};
      auto tool_pose = arrayToMsg(m_hw_handle.getToolPoseRaw());
      if (m_comau_robot_status_pub->trylock())
      {
        m_comau_robot_status_pub->msg_.header.stamp = time;
        m_comau_robot_status_pub->msg_.joint_positions = joints;
        m_comau_robot_status_pub->msg_.status.status = status;
        m_comau_robot_status_pub->msg_.tool_pose = tool_pose;
        m_comau_robot_status_pub->unlockAndPublish();
        m_last_publish_time += ros::Duration(1 / m_publish_rate);
      }
    }
  }
} // namespace hrr_controllers
PLUGINLIB_EXPORT_CLASS(hrr_controllers::TP5StateController, controller_interface::ControllerBase);