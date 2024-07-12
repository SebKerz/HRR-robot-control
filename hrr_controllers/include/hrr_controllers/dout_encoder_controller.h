/**
 * @file screwdriver_controller.h
 * @author TUM
 * @brief  DOUT digital encoder controller.h
 * @version 0.2
 * @date 25-09-2021
 *
 * Expects to control a 4-bit address encoder via DOUT, i.e.
 *
 * @copyright Copyright (c) 2021
 */
#ifndef HRR_CONTROLLERS_DOUT_ENCODER_CONTROLLER_H
#define HRR_CONTROLLERS_DOUT_ENCODER_CONTROLLER_H

#pragma once

#include <sstream>
#include <controller_interface/controller.h>
#include <comau_driver/comau_hw_interfaces/digital_io_interface.h>
#include <comau_msgs/Digital.h>
#include <realtime_tools/realtime_publisher.h>
#include <hrr_controllers/core/utils.h>
#include <hrr_msgs/SetEncoderState.h>
#include <hrr_msgs/GetDoutControllerState.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace hrr_controllers {


  class DoutEncoderController :
      public controller_interface::Controller<hardware_interface::comau::DigitalPinCommandInterface> {
  public:

    bool init(hardware_interface::comau::DigitalPinCommandInterface *hw, ros::NodeHandle &ctrl_nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void starting(const ros::Time &time) override;

    void stopping(const ros::Time &time1) override;

    inline static const std::string m_log_name{"DoutEncoderController"};

  private:

    bool get_controller_state_srv(hrr_msgs::GetDoutControllerStateRequest &req,
                                  hrr_msgs::GetDoutControllerStateResponse &res);

    void reset();

    bool set_encoder_state_srv(hrr_msgs::SetEncoderStateRequest &req, hrr_msgs::SetEncoderStateResponse &res);

    typedef hardware_interface::comau::DigitalPinCommandHandle DigPinHandle;

    ros::NodeHandle m_nh;
    std::vector<DigPinHandle> m_encoder_pins{};
    std::vector<ros::ServiceServer> m_ros_srvs{};
    std::map<std::string, int> m_state_map{};
    std::vector<int> m_legal_out{};
    std::shared_ptr<DigPinHandle> m_enable_encoder_pin{nullptr};
    ros::Time m_set_time{};
    ros::Duration m_timeout{0};
    std::string m_frame_id{};
    int m_encoder_size{3};
    int m_current_out{-1};
    int m_desired_out{-1};
    int8_t m_tool_type{0};
  };
}
#endif //HRR_CONTROLLERS_DOUT_ENCODER_CONTROLLER_H
