/**
 * @file hrr_cobot_hw.h
 * @author TUM
 * @brief 
 * @version 0.1
 * @date 25-01-2021
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef ROS_CONTROL__HRR_COBOT_HARDDWARE_INTERFACE_H
#define ROS_CONTROL__HRR_COBOT_HARDDWARE_INTERFACE_H

#include <sstream>
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <pluginlib/class_loader.h>
#include <force_torque_plugin/force_torque_plugin.h>
#include <comau_driver/comau_robot_hw.h>
#include <hrr_common/utils.h>

namespace hrr_cobot {

    
    
    /**
     * @brief The HR-Recycler Cobot Hardware interface. It extends the default Comau hardare interface by additional 
     * sensor and endeffector handles.
     */    
    class HrrCobotHW : public hardware_interface::comau::ComauRobotHW {
    public:

        HrrCobotHW() = default;        

        bool init(ros::NodeHandle &nh, ros::NodeHandle &nh_local) final;

        void read(const ros::Time &time, const ros::Duration &period) final;
        
        bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                           const std::list<hardware_interface::ControllerInfo> &stop_list) override;
    private:                
        // Name of this class        
        const std::string m_log_name{"Hr-Recycler-Cobot"};        
        std::string m_name{"hrr_cobot_hardware_interface"};
        std::shared_ptr<hardware_interface::ForceTorquePlugin> m_ft_sensor{};        
    };
    
} // namespace hardware_interface::comau

#endif


