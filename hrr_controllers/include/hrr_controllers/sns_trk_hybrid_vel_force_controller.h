/**
 * @file sns_trk_hybrid_vel_force_controller.h
 * @author TUM
 * @brief  hybrid velocity force controller
 * @version 0.2
 * @date 06-05-2021
 *
 * @copyright Copyright (c) 2021
 */
#ifndef HRR_CONTROLLERS_HYBRID_FORCE_VELOCITY_CONTROLLER_H
#define HRR_CONTROLLERS_HYBRID_FORCE_VELOCITY_CONTROLLER_H

#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <realtime_tools/realtime_publisher.h>

// geometry handlers
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/SetBool.h>
#include <eigen_conversions/eigen_msg.h>

//controller interfaces
#include <controller_interface/controller.h>

// (meta) package includes
#include <hrr_common/conversions.h>
#include <hrr_msgs/HybridForceVelocityCmdStamped.h>
#include <hrr_msgs/HybridForceVelocityControlSelect.h>
#include <hrr_controllers/GainsConfig.h>
#include <hrr_controllers/core/params.h>
#include <hrr_controllers/core/sns_trk_api.h>
#include <hrr_controllers/sns_trk_velocity_controller.h>

namespace hrr_controllers {


    struct CompliantControllerParams: public SnsTrkVelRosParams {
        std::string wrench_cmd_topic_name{"B_ft_des"};
        std::string wrench_cur_topic_name{"B_ft_cur"};
        std::string control_select_topic_name{"S"};
        std::string full_cmd_topic_name{"cmd"};
        std::string set_calibrated_srv_name{"acknowledge_calibration"};
        double f_max{80.0}, m_max{80.0};
        bool publish_ft{true};

        inline explicit CompliantControllerParams(const ros::NodeHandle &nh) : SnsTrkVelRosParams{nh}{
            valid = getParam<double>(nh, "SnsTrkVelocityController", "loop_hz", loop_rate);
            if (valid)
                valid = getParam<std::string>(nh, "SnsTrkVelocityController", "sns_trk_hw_name", sns_trk_hw_name);
            nh.param<std::string>("wrench_cur_topic_name", wrench_cur_topic_name, wrench_cur_topic_name);
            nh.param<std::string>("wrench_cmd_topic_name", wrench_cmd_topic_name, wrench_cmd_topic_name);
            nh.param<std::string>("control_select_topic_name", control_select_topic_name, control_select_topic_name);
            nh.param<std::string>("full_cmd_topic_name", full_cmd_topic_name, full_cmd_topic_name);
            nh.param<std::string>("set_calibrated_srv_name", set_calibrated_srv_name, set_calibrated_srv_name);
            nh.param<double>("f_max", f_max, f_max);
            nh.param<double>("m_max", m_max, m_max);
            nh.param<bool>("publish_ft", publish_ft, publish_ft);
        }

        [[nodiscard]] std::string print_compliant_params() const {
            std::stringstream ss{};
            ss << "\n\twrench_cmd_topic_name:     \t" << wrench_cmd_topic_name
               << "\n\twrench_cur_topic_name:     \t" << wrench_cur_topic_name
               << "\n\tcontrol_select_topic_name: \t" << control_select_topic_name
               << "\n\tfull_cmd_topic_name:       \t" << full_cmd_topic_name
               << "\n\tset_calibrated_srv_name:   \t" << set_calibrated_srv_name
               << "\n\tpublish_ft:                \t" << publish_ft
               << "\n\tf_max [N]:                 \t" << f_max
               << "\n\tm_max [Nm]:                \t" << m_max;
            return print_params() + ss.str();
        }

    };


    template<class T>
    class CompliantController : public controller_interface::Controller<T> {
    public:
        CompliantController() = default;

        bool init(T *hw, ros::NodeHandle &nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void starting(const ros::Time &time) override;

        void stopping(const ros::Time &time) override;

        GenericSnsTrkHandle m_hw_handle;

        inline const static std::string m_log_name{"SnsTrkHybridForceVelocityController"};

    private:
        // ROS handles
        ros::NodeHandle m_nh{};
        std::vector<ros::Subscriber>     m_ros_subs{};
        std::vector<ros::ServiceServer>  m_ros_srvs{}; 
        std::vector<RtWrenchPubPtr>      m_ft_pubs{};
        std::shared_ptr<tf2_ros::Buffer> m_tf_buffer{};
        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{};
        dynamic_reconfigure::Server<hrr_controllers::GainsConfig> m_dyn_rfc_server;
        dynamic_reconfigure::Server<hrr_controllers::GainsConfig>::CallbackType m_dyn_rcf_cb;

        // velocity values
        Eigen::Vector3d m_vel_p_des{ZeroVec},
                        m_vel_rot_des{ZeroVec};

        // transformations
        Eigen::Vector3d    m_E_p_EC{ZeroVec};
        Eigen::Quaterniond m_quat_E_C{ZeroQuat};

        // wrench values
        Eigen::Vector3d m_B_force_cur{ZeroVec},
                        m_B_force_des{ZeroVec},
                        m_B_torque_cur{ZeroVec},
                        m_B_torque_des{ZeroVec};

        // control selectors & final values
        std::array<bool,   6> m_S_vel{false};
        std::array<bool,   6> m_S_force{false};
        std::array<double, 6> m_K_p_force{1e-4};
        std::array<double, 6> m_K_i_force{0.0};

        // ROS time handles
        ros::Time m_last_msg{},
                  m_last_ft_read{},
                  m_last_write{};

        // reference frames
        std::string m_reference_frame{"base_link"},
                    m_tcp_frame{"tcp_controller"},
                    m_wrench_frame{"base_link"},
                    m_prev_control_frame{""};

        // limits
        double m_F_max{80.0}, m_M_max{80.0};
        double m_dt{0.0};
        double m_dt_timeout{0.1};
        double m_ee_v_max{0.1};
        double m_ee_omega_max{0.2};
        double m_force_precision{1.0};
        double m_K_p_f_max{1e-3}, m_K_p_t_max{1e-2};

        //flags
        bool m_debug{false}, m_keep_base_orientation{true},
             m_print_warning{true}, m_print_cb_warning{true}, 
             m_print_FT_warnings{true}, m_print_S_warning{true},
             m_publish_ft_data{false},
             m_ft_calibrated{false};

        /**
         * @brief Set the all values and commands to zero
         */
        void setZero();

        void setReferenceFrame(const std::string &frame_id);

        std::tuple<Eigen::Vector3d, Eigen::Vector3d> wrench2base_frame(
                const std::string &reference_frame_id,  const geometry_msgs::Wrench &msg);

        void setGainsCb(hrr_controllers::GainsConfig &gain_config, uint32_t level);

        void wrenchReadCb(const geometry_msgs::WrenchStampedConstPtr &msg);

        void wrenchCmdCb(const geometry_msgs::WrenchStampedConstPtr &msg);

        void twistCmdCb(const geometry_msgs::TwistStampedConstPtr &msg);

        void controlSelectHelper(const std::vector<uint8_t> &S_force, const std::vector<uint8_t> &S_vel);

        void controlSelectCb(const hrr_msgs::HybridForceVelocityControlSelectConstPtr &msg);

        void fullCmdCb(const hrr_msgs::HybridForceVelocityCmdStampedConstPtr &msg);

        bool setCalibratedFlagSrv(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    };


} // namespace hrr_controllers
#endif //HRR_CONTROLLERS_HYBRID_FORCE_VELOCITY_CONTROLLER_H
