/**
 * @file sns_trk_velocity_controller.h
 * @author TUM
 * @brief  Sensor-Track velocity force controller
 * @version 0.2
 * @date 26-01-2021
 *
 * @copyright Copyright (c) 2021
 */
#ifndef HRR_CONTROLLERS_CARTESIAN_VELOCITY_CONTROLLER_H
#define HRR_CONTROLLERS_CARTESIAN_VELOCITY_CONTROLLER_H

#pragma once
#include<sstream>

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

// geometry handlers
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen_conversions/eigen_msg.h>

//controller interfaces
#include <controller_interface/controller.h>

// (meta) package includes
#include <hrr_common/conversions.h>
#include <hrr_msgs/SnsTrkTwistCmd.h>
#include <hrr_controllers/core/params.h>
#include <hrr_controllers/core/sns_trk_api.h>


namespace hrr_controllers {

    inline void clipVel(Eigen::Vector3d vel, const double &max_v) {
        auto v_norm = vel.norm();
        if (v_norm > max_v) {
            ROS_DEBUG("clip velocity by factor %.3f", max_v / v_norm);
            vel = vel * max_v / v_norm;
        } else {
            vel = hrr_cobot::clip(vel, -max_v, max_v);
        }
    }

    template<class T>
    class SnsTrkVelocityController : public controller_interface::Controller<T> {
    public:
        SnsTrkVelocityController() = default;

        bool init(T *hw, ros::NodeHandle &nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void starting(const ros::Time &time) override;

        void stopping(const ros::Time &time) override;

        GenericSnsTrkHandle m_hw_handle;

        inline const static std::string m_log_name{"SnsTrkVelocityController"};

    private:
        ros::NodeHandle m_nh{};
        std::vector<ros::Subscriber> m_cmd_subs{};
        std::shared_ptr<tf2_ros::Buffer> m_tf_buffer{};
        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{};
        Eigen::Vector3d m_vel_p_des{Eigen::Vector3d::Zero()};
        Eigen::Vector3d m_vel_rot_des{Eigen::Vector3d::Zero()};
        Eigen::Vector3d m_E_p_EC{ZeroVec};
        Eigen::Quaterniond m_quat_E_C{ZeroQuat};
        ros::Time m_last_msg{};
        ros::Time m_last_write{};
        std::string m_reference_frame{"base_link"};
        std::string m_tcp_frame{"tcp_controller"};
        std::string m_prev_control_frame{};
        double m_dt{0.0};
        double m_dt_timeout{0.2};
        double m_ee_v_max{0.1};
        double m_ee_omega_max{0.2};
        bool m_keep_base_orientation{true}, m_debug{false}, 
             m_print_warning{true}, m_print_cb_warning{true};

        void twistCmdCb(const geometry_msgs::TwistStampedConstPtr &msg);

        void snsTrkTwistCmdCb(const hrr_msgs::SnsTrkTwistCmdPtr &msg);

        void setZero();

    };


    struct SnsTrkVelRosParams {
        std::string sns_trk_hw_name;
        std::string cmd_topic_name{"x_dot_des_tcp"};
        std::string sns_trk_cmd_topic_name{"x_dot_des"};
        double loop_rate{0.0};
        double timeout{0.2};
        double ee_v_max{0.1};
        double ee_omega_max{0.2};
        bool debug{true};
        bool valid{true};

        inline explicit SnsTrkVelRosParams(const ros::NodeHandle &nh) {
            valid = getParam<double>(nh, "SnsTrkVelocityController", "loop_hz", loop_rate);
            if (valid)
                valid = getParam<std::string>(nh, "SnsTrkVelocityController", "sns_trk_hw_name", sns_trk_hw_name);
            nh.param<std::string>("cmd_topic_name", cmd_topic_name, cmd_topic_name);
            nh.param<std::string>("sns_trk_cmd_topic_name", sns_trk_cmd_topic_name, sns_trk_cmd_topic_name);
            nh.param<double>("ee_v_max", ee_v_max, ee_v_max);
            nh.param<double>("dead_man_timeout", timeout, timeout);
            nh.param<bool>("debug", debug, debug);
        }

        [[nodiscard]] std::string print_params() const {
            std::stringstream ss{};
            ss << "\n\tcmd_topic_name:     \t" << cmd_topic_name
               << "\n\tsns_trk_hw_name:    \t" << sns_trk_hw_name
               << "\n\tdebug_out:          \t" << debug
               << "\n\tloop_rate [ Hz]:    \t" << loop_rate
               << "\n\tee_v_max  [m/s]:    \t" << ee_v_max
               << "\n\ttimeout   [ s ]:    \t" << timeout;
            return ss.str();
        }

    };

} // namespace hrr_controllers
#endif //HRR_CONTROLLERS_CARTESIAN_VELOCITY_CONTROLLER_H
