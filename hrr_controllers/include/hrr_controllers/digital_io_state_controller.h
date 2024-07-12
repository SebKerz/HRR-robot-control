/**
 * @file digital_io_state_controller.h
 * @author TUM
 * @brief   COMAU DOUT/DIN state controller
 * @version 0.1
 * @date 27-01-2022.
 *
 * @copyright Copyright (c) 2022
 */
#ifndef HRR_CONTROLLERS_DIGITAL_IO_STATE_CONTROLLER_H
#define HRR_CONTROLLERS_DIGITAL_IO_STATE_CONTROLLER_H

#pragma once

#include <sstream>
#include <controller_interface/controller.h>
#include <comau_driver/comau_hw_interfaces/digital_io_interface.h>
#include <comau_msgs/Digital.h>
#include <comau_msgs/IOStates.h>
#include <realtime_tools/realtime_publisher.h>


namespace hrr_controllers {


  class DigitalIOStateController
      : public controller_interface::Controller<hardware_interface::comau::DigitalIOStateInterface> {
  public:
    bool init(hardware_interface::comau::DigitalIOStateInterface *hw, ros::NodeHandle &ctrl_nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void starting(const ros::Time &time) override {
      m_prev_pub = time;
      m_prev_pub -= ros::Duration(2.0 * m_publish_rate);
    };

    inline static const std::string m_log_name = "DigitalIOStateController";

  private:
    typedef realtime_tools::RealtimePublisher<comau_msgs::IOStates> RtIOsPub;
    typedef hardware_interface::comau::DigitalIOStateHandle DigStateHandle;

    ros::NodeHandle m_nh;
    std::pair<DigStateHandle, DigStateHandle> m_io_handles{};
    std::shared_ptr<RtIOsPub> m_io_pub{};
    ros::Time m_prev_pub{};
    double m_publish_rate{0.0};

  };


}
#endif //HRR_CONTROLLERS_DIGITAL_IO_STATE_CONTROLLER_H
