/**
 * @file ft_state_controller.h
 * @author TUM
 * @brief   F/T state controller
 * @version 0.2
 * @date 25-09-2021
 *
 * @copyright Copyright (c) 2021
 */
#ifndef HRR_CONTROLLERS_SENSOR_CALIBRATIONS_H
#define HRR_CONTROLLERS_SENSOR_CALIBRATIONS_H

#pragma once

#include <numeric>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <eigen_conversions/eigen_msg.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hrr_msgs/ToolCom.h>
#include <hrr_controllers/core/params.h>


namespace hrr_controllers {

    const Eigen::Vector3d gravity_B{0.0, 0.0, -9.81};
    const std::string ft_raw{"ft_raw"};
    const std::string ft_no_bias{"ft_bias_free"};
    const std::string ft_filtered{"ft_filtered"};
    const std::string ft_tool_comp{"ft_tool_comp"};

    template<typename T>
    bool add_to_buffer(const T value, std::vector<T> buffer, const int32_t &buf_max = 100) {
        if (buffer.size < buf_max) {
            buffer.push_back(value);
            return buffer.size() == buf_max;
        }
        return true;
    }

    template<typename T>
    T get_mean(const std::vector<T> buffer, T zero_value) {
        if (buffer.size() > 0) {
            auto sum = std::accumulate(buffer.begin(), buffer.end(), zero_value,
                                       [&buffer](auto &e1, auto &e2) { return e1 + e2 / buffer.size(); });
            return sum / buffer.size();
        }
        return zero_value;
    }

    inline geometry_msgs::Wrench toMsg(const std::pair<Eigen::Vector3d, Eigen::Vector3d> &wrench_pair) {
        geometry_msgs::Wrench msg{};
        tf::vectorEigenToMsg(wrench_pair.first, msg.force);
        tf::vectorEigenToMsg(wrench_pair.second, msg.torque);
        return msg;
    }


        struct SensorCalibParams{
        std::string sensor_hw_name;
        std::string ft_com_topic_name{"center_of_mass"};
        std::string ft_bias_topic_name{"sensor_bias"};
        double publish_rate{0.0};
        int32_t buf_size{200};
        bool valid{true};

        explicit SensorCalibParams(const ros::NodeHandle &nh);

        [[nodiscard]] std::string print_params() const;
    };


    struct SensorCalibStateControllerParams{
        std::string raw_topic{"ft_raw"};
        std::string filtered_topic{"ft_filtered"};
        std::string no_bias_topic{"ft_no_bias"};
        std::string compensated_topic{"ft_compensated"};
        double publish_rate{0.0};
        bool publish_raw{true};
        bool publish_filtered{false};
        bool publish_bias_free{true};
        bool publish_compensated{false};
        bool valid{true};

        explicit SensorCalibStateControllerParams(const ros::NodeHandle &nh);

        [[nodiscard]] std::string print_params() const;
    };


   class CalibrateSensor {
    public:
        CalibrateSensor() = default;

        void rosInit(hardware_interface::ForceTorqueSensorInterface *hw, ros::NodeHandle &nh);

        void processData();

        void resetBuffer();

        [[nodiscard]] std::string getSensorFrame() const { return m_ft_handle.getFrameId(); };

        [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d> getRaw() const { return m_raw; };

        [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d> getWrench() const;

        [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d> getWrench(const Eigen::Affine3d &T_base_sensor) const;

        [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d> getWrench(int &n_filter) const;

        [[nodiscard]] Eigen::Vector3d getToolCoM() const { return m_tool_com; }

        [[nodiscard]] double getToolMass() const { return  m_mass; }

        [[nodiscard]] bool isInitialized() const { return m_initialized; }

        [[nodiscard]] std::pair<Eigen::Vector3d, Eigen::Vector3d> getBias() const { return m_bias; }
        
        inline static const std::string m_log_name = "FtStateController";

    private:
        std::vector<ros::Subscriber> m_ros_subs{};
        std::vector<ros::ServiceServer> m_ros_srvs{};
        ros::NodeHandle m_nh{};
        hardware_interface::ForceTorqueSensorHandle m_ft_handle;
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> m_bfr{};
        std::pair<Eigen::Vector3d, Eigen::Vector3d> m_bias{};
        std::pair<Eigen::Vector3d, Eigen::Vector3d> m_raw{};
        Eigen::Vector3d m_tool_com{};
        int32_t m_roll_buf_idx{0};
        int32_t m_buf_size{0};
        double m_mass{0.0};
        bool m_initialized{false};

        void update_params();

        bool updateFromParameters(const std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        void setToolComCb(const hrr_msgs::ToolComPtr &msg);

        void setSensorBiasCb(const geometry_msgs::WrenchPtr &msg);
    };

    class CalibratedFTStateController
            : public controller_interface::Controller<hardware_interface::ForceTorqueSensorInterface>,
              public CalibrateSensor {
    public:
        bool init(hardware_interface::ForceTorqueSensorInterface *hw, ros::NodeHandle &nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void starting(const ros::Time &time) override;

        inline const static std::string m_log_name ="FTCalibStateController";

    private:
        typedef realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> RtWrenchPub;
        typedef std::shared_ptr<RtWrenchPub> RtWrenchPubPtr;
        std::vector<RtWrenchPubPtr> m_ros_pubs{};
        std::vector<ros::Time> m_last_publish_times{};
        std::map<std::string, int> m_pub_idxs{};
        std::set<std::string> m_active_pubs{};
        Eigen::Affine3d m_cur_pose{};
        double m_publish_rate{};
        int m_window_size{};

        void publish_helper(const int &id, const geometry_msgs::Wrench &wrench_msg, const ros::Time &time);
    };
}

#endif