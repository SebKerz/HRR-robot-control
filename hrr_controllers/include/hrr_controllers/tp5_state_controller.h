/**
 * @file tp5_state_controller.h
 * @author TUM
 * @brief  COMAU TP5 state controller
 * @version 0.2
 * @date 26-01-2021
 *
 * @copyright Copyright (c) 2021
 */
#ifndef HRR_CONTROLLERS_TP5_STATE_CONTROLLER_H
#define HRR_CONTROLLERS_TP5_STATE_CONTROLLER_H

#include <comau_driver/comau_hw_interfaces/tp5_state_interface.h>
#include <comau_driver/helpers.h>
#include <comau_msgs/CartesianPose.h>
#include <comau_msgs/ComauRobotStatus.h>
#include <comau_msgs/TP5State.h>
#include <controller_interface/controller.h>
#include <hrr_controllers/core/params.h>
#include <realtime_tools/realtime_publisher.h>

namespace hrr_controllers
{
  class TP5StateController : public controller_interface::Controller<hardware_interface::comau::TP5StateInterface>
  {
  public:
    bool init(hardware_interface::comau::TP5StateInterface *hw, ros::NodeHandle &nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void starting(const ros::Time &time) override
    {
      m_last_publish_time = time;
    }

    inline static const std::string m_log_name{"TP5StateController"};

  private:
    typedef realtime_tools::RealtimePublisher<comau_msgs::TP5State> TP5Pub;

    hardware_interface::comau::TP5StateHandle m_hw_handle{};
    std::unique_ptr<TP5Pub> m_comau_robot_status_pub{};
    ros::Time m_last_publish_time{};
    double m_publish_rate{0.0};

  }; // TP5 State Controller
} // namespace hrr_controllers
#endif // HRR_CONTROLLERS_TP5_STATE_CONTROLLER_H
