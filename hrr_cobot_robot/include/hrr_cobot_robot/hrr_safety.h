#ifndef HRR_COBOT_SAFETY_H
#define HRR_COBOT_SAFETY_H

// default
#include <iostream>
#include <chrono>
#include <ctime>
#include <memory>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_srvs/Trigger.h>
#include <hrr_msgs/DeadMan.h>
#include <hrr_msgs/GetDeadManState.h>
#include <hrr_common/utils.h>
#include <hrr_cobot_robot/hrr_cobot_hw.h>



namespace hrr_cobot{
    class SafetyWatchDog{

        public:
            SafetyWatchDog() = default;
            ~SafetyWatchDog() = default;
            
            SafetyWatchDog(ros::NodeHandle &nh, const std::string &param_ns) {assert(init_ros(nh, param_ns));};

            [[nodiscard]] bool isAlive () const {return m_alive;};
            
            [[nodiscard]] bool init_ros(ros::NodeHandle &nh, const std::string &param_ns);

            [[nodiscard]] bool update(std::shared_ptr<HrrCobotHW> &hrr_cobot_hw, 
                                      std::shared_ptr<controller_manager::ControllerManager> &controller_manager);

        private:
            ros::Subscriber m_dead_man_signal_sub{};
            ros::ServiceServer m_dead_man_state_srv{};
            std::shared_ptr<ros::ServiceClient> m_trigger_tool_srv{};
            struct timespec m_last_dead_man{};
            ros::Duration m_dead_man_timeout{0.2};
            const std::string m_log_name{"Hrr-Safety"};;
            std::string m_param_ns;
            std::map<comau_driver::ControlMode, std::vector<std::string>> m_black_list_controllers;
            bool m_dead_man{false};
            bool m_alive{false};
            
            void dead_man_cb(const hrr_msgs::DeadMan &msg);            
            bool dead_man_state_srv(hrr_msgs::GetDeadManState::Request &req,
                                    hrr_msgs::GetDeadManState::Response &res);

    };//SafetyWatchDog
}//namespace
#endif