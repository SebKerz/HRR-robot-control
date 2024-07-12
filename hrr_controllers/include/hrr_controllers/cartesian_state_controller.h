/**
 * @file cartesian_state_controller.h
 * @author TUM
 * @brief   COMAU Cartesian state controller
 * @version 0.1
 * @date 26-01-2021.
 *
 * @copyright Copyright (c) 2021
 */
#ifndef HRR_CONTROLLERS_CARTESIAN_SATE_CONTROLLER_H
#define HRR_CONTROLLERS_CARTESIAN_SATE_CONTROLLER_H

#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <controller_interface/controller.h>
#include <comau_msgs/CartesianPose.h>
#include <comau_msgs/CartesianPoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <realtime_tools/realtime_publisher.h>
#include <comau_driver/comau_hw_interfaces/cartesian_state_interface.h>
#include <hrr_controllers/core/params.h>
#include <hrr_controllers/core/utils.h>


namespace hrr_controllers {


    struct CStateRosParams {
        std::string hw_name{""};
        std::string raw_topic_name{"comau_cartesian_data"};
        std::string sns_tcp_frame{"sns_tcp"};
        std::string tf_topic_name{"/tf"};
        double publish_rate{0.0};
        bool pub_tf{true};
        bool pub_raw{true};
        bool valid{true};

        explicit CStateRosParams(const ros::NodeHandle &nh);

        [[nodiscard]] std::string print_params() const;
    };


    class CartesianStateController: public controller_interface::Controller<hardware_interface::comau::CartesianStateInterface> {
    public:
        bool init(hardware_interface::comau::CartesianStateInterface *hw,
                  ros::NodeHandle &nh, ros::NodeHandle &ctrl_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void starting(const ros::Time &time) override;

        inline static const std::string m_log_name = "CartesianStateController";

    private:
        typedef realtime_tools::RealtimePublisher<comau_msgs::CartesianPoseStamped> RtCartPosePub;

        bool update_params();

        ros::NodeHandle m_nh;
        hardware_interface::comau::CartesianStateHandle m_hw_handle{};
        Eigen::Vector3d    m_tcp_offset{ZeroVec};
        Eigen::Quaterniond m_tcp_rotation{ZeroQuat};
        std::shared_ptr<RtTfPub> m_tf_pose_pub{};
        std::shared_ptr<RtCartPosePub> m_comau_pose_pub{};
        std::pair<bool, ros::Time> m_pub_tf_data{};
        std::pair<bool, ros::Time> m_pub_pose_data{};
        std::string m_tcp_frame{"sns_tcp"};
        double m_publish_rate;


    };





}
#endif //HRR_CONTROLLERS_CARTESIAN_SATE_CONTROLLER_H
