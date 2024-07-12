/**
 * @file dout_controller.h
 * @author TUM
 * @brief   COMAU DOUT controller
 * @version 0.1
 * @date 27-01-2022.
 *
 * @copyright Copyright (c) 2022
 */
#ifndef HRR_CONTROLLERS_DUAL_PIN_CONTROLLER_H
#define HRR_CONTROLLERS_DUAL_PIN_CONTROLLER_H

#pragma once

#include <sstream>
#include <controller_interface/controller.h>
#include <comau_driver/comau_hw_interfaces/digital_io_interface.h>
#include <comau_msgs/Digital.h>
#include <hrr_msgs/GetDoutControllerState.h>
#include <hrr_msgs/SetPins.h>
#include <std_srvs/Trigger.h>
#include <hrr_controllers/core/utils.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace hrr_controllers {


  class DigitalPinController
      : public controller_interface::Controller<hardware_interface::comau::DigitalPinCommandInterface> {
  public:
    bool init(hardware_interface::comau::DigitalPinCommandInterface *hw, ros::NodeHandle &ctrl_nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void starting(const ros::Time &time) override;

    void stopping(const ros::Time &time1) override;

    inline static const std::string m_log_name{"DOUTController"};

  private:
    typedef hardware_interface::comau::DigitalPinCommandHandle DigPinHandle;

    void reset();

    bool invert_srv(std_srvs::TriggerRequest  &req, std_srvs::TriggerResponse &res);

    bool reset_srv(std_srvs::TriggerRequest  &req, std_srvs::TriggerResponse &res);

    bool set_pins_srv(hrr_msgs::SetPinsRequest &req, hrr_msgs::SetPinsResponse &res);


    bool get_controller_state_srv(hrr_msgs::GetDoutControllerStateRequest &req,
                                  hrr_msgs::GetDoutControllerStateResponse &res);


    std::vector<ros::ServiceServer> m_ros_srvs{};
    ros::NodeHandle m_nh;
    std::map<int, DigPinHandle> m_pin_handles{};
    std::map<int, bool> m_current_values{};
    std::map<int, bool> m_desired_values{};
    std::map<int, bool> m_start_values{};
    ros::Time m_set_time{};
    ros::Duration m_timeout{0};
    std::string m_frame_id{};
    bool m_enabled{false};
    bool m_error{false};
  };


}
#endif //HRR_CONTROLLERS_DUAL_PIN_CONTROLLER_H
