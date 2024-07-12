//
// Created by gabler on 27.01.21.
//

#include <hrr_controllers/sns_trk_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

namespace hrr_controllers
{

    template<class T>
    void SnsTrkVelocityController<T>::update(const ros::Time &time, const ros::Duration &period) {
        if (m_last_write + ros::Duration(m_dt) > time) {
            ROS_DEBUG_NAMED(m_log_name, "next time step %f (t_prev=%f, dt = %f) > current %f",
                            (m_last_write + ros::Duration(m_dt)).toSec(),
                            m_last_write.toSec(), m_dt, time.toSec());
            return;
        }
        // Limit Velocity
        if (m_last_msg + ros::Duration(m_dt_timeout) < time) {
            if (m_print_warning) {
                ROS_DEBUG("[%s] to_zero", m_log_name.c_str());
                m_print_warning = false;
            }
            setZero();
        } else {
            // transform velocity according to current reference frame / center of orientation
            Eigen::Matrix<double, 6, 1> B_v_cmd{};
            Eigen::Quaterniond B_quat_E{};
            tf::quaternionMsgToEigen(getRotation(m_hw_handle), B_quat_E);
            Eigen::Matrix3d R_B_C{B_quat_E * m_quat_E_C};
            if (m_keep_base_orientation){
                B_v_cmd.head(3) = m_vel_p_des;
                B_v_cmd.tail(3) = m_vel_rot_des;
            } else {
                B_v_cmd.head(3) = R_B_C * m_vel_p_des;
                B_v_cmd.tail(3) = R_B_C * m_vel_rot_des;
            }
            B_v_cmd.head(3) += (B_quat_E.matrix() * m_E_p_EC).cross(R_B_C * m_vel_rot_des);
            if (m_vel_rot_des.norm() > 0.0) {
                ROS_DEBUG_STREAM_THROTTLE_NAMED(0.5, m_log_name, 
				"transformed " << m_vel_rot_des.transpose() << "to: " << B_v_cmd.head(3).transpose()
                );
            }
            if (B_v_cmd.norm() > m_ee_v_max + m_ee_omega_max && B_v_cmd.norm() > 1e-7) {
                B_v_cmd = B_v_cmd * (m_ee_v_max + m_ee_omega_max) / B_v_cmd.norm();
            } else {
                B_v_cmd.head(3) = hrr_cobot::clip(B_v_cmd.head(3), -m_ee_v_max, m_ee_v_max);
                B_v_cmd.tail(3) = hrr_cobot::clip(B_v_cmd.tail(3), -m_ee_omega_max, m_ee_omega_max);
            }
            if (!setEigenPose(m_hw_handle,
                              Eigen::Vector3d{B_v_cmd.head(3) * m_dt},
                              hrr_cobot::zeroOrderDifference(B_v_cmd.tail(3), m_dt))) {
                ROS_WARN_THROTTLE_NAMED(0.2, m_log_name, "failed to set pose to robot");
            }
            m_last_write += ros::Duration{m_dt};
            // Integrate and send to hardware
            if (m_debug) {
                auto[valid, p, q]  = getPoseCmd(m_hw_handle);
                if (valid) {
                    ROS_DEBUG_THROTTLE_NAMED(0.1, m_log_name,
                                    "[%s] pos-command: [%.2f, %.2f, %.2f]\nrot-command: [%.3f, %.3f, %.3f, %.3f]",
                                    m_log_name.c_str(),
                                    p.x * 1e3, p.y * 1e3, p.z * 1e3,
                                    q.x, q.y, q.z, q.w);
                } else {
                    auto[valid2, cmd]  = getPoseArrayCmd(m_hw_handle);
                    if (valid2) {
                        ROS_DEBUG_THROTTLE_NAMED(0.1, m_log_name, "[%s] command: [%.2f, %.2f, %.2f, %.3f, %.3f, %.3f] (1e-3)",
                                        m_log_name.c_str(),
                                        cmd[0] * 1e3, cmd[1] * 1e3, cmd[2] * 1e3,
                                        cmd[3] * 1e3, cmd[4] * 1e3, cmd[5] * 1e3);
                    }
                }
            }
            m_print_warning = true;
        }

    }

    template<class T>
    void SnsTrkVelocityController<T>::starting(const ros::Time &time) {
        setZero();
        m_reference_frame = getReferenceFrame(m_hw_handle);
        m_tcp_frame = getTargetFrame(m_hw_handle);
        m_last_write = time;
        m_print_warning = true;
        m_print_cb_warning = true;
    }

    template<class T>
    void SnsTrkVelocityController<T>::stopping(const ros::Time &time) {
        setZero();
    }

    template<class T>
    void SnsTrkVelocityController<T>::setZero() {
        m_vel_p_des.setZero();
        m_vel_rot_des.setZero();
        m_E_p_EC.setZero();
        m_quat_E_C.setIdentity();
        double dt{0.0};
        if (!setEigenPose(m_hw_handle, Eigen::Vector3d{m_vel_p_des * dt}, Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0})) {
            ROS_WARN("failed to set pose to robot");
        }
    }

    template<class T>
    void SnsTrkVelocityController<T>::twistCmdCb(const geometry_msgs::TwistStampedConstPtr &msg) {
        tf::vectorMsgToEigen(msg->twist.linear, m_vel_p_des);
        tf::vectorMsgToEigen(msg->twist.angular, m_vel_rot_des);
        if (msg->header.frame_id.empty()) {
            if (m_print_cb_warning) {
                ROS_WARN("[%s] empty frame_id received. Assuming default control: %s w.r.t. %s",
                         m_log_name.c_str(),
                         m_tcp_frame.c_str(),
                         m_reference_frame.c_str());
                m_print_cb_warning = false;
            }
            m_E_p_EC.setZero();
            m_quat_E_C.setIdentity();
            m_keep_base_orientation = true;
        } else if ((msg->header.frame_id != m_reference_frame) and (msg->header.frame_id != m_tcp_frame)) {
            try {
                auto T_E_C = m_tf_buffer->lookupTransform(m_tcp_frame, msg->header.frame_id, ros::Time(0));
                tf::vectorMsgToEigen(T_E_C.transform.translation, m_E_p_EC);
                tf::quaternionMsgToEigen(T_E_C.transform.rotation, m_quat_E_C);
                m_keep_base_orientation = m_quat_E_C.isApprox(Eigen::Quaterniond::Identity());
                if (m_prev_control_frame != msg->header.frame_id){
                    m_prev_control_frame = msg->header.frame_id;
                    if (m_keep_base_orientation){
                        ROS_INFO_STREAM_THROTTLE_NAMED(0.5,
                                m_log_name, "[" << m_log_name << "] received control frame (" << m_prev_control_frame
                                                << ") C_p_EC:= " << m_E_p_EC.transpose() << "without rotation!");
                    } else {
                        ROS_INFO_STREAM_THROTTLE_NAMED(0.5,
                                m_log_name, "[" << m_log_name << "] received control frame (" << m_prev_control_frame
                                                << ") C_p_EC:= " << m_E_p_EC.transpose() << "R_E_C:="
                                                << m_quat_E_C.matrix());
                    }
                } else {
                    ROS_DEBUG_STREAM_THROTTLE_NAMED(0.5,
                            m_log_name, "[" << m_log_name << "] received control frame ("<<m_prev_control_frame
                                            <<") C_p_EC:= " <<   m_E_p_EC.transpose() << "R_E_C:=" << m_quat_E_C.matrix());
                }
            } catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE_NAMED(0.5, m_log_name, "[%s] Could NOT get transformation from %s to %s",
                         m_log_name.c_str(), msg->header.frame_id.c_str(), m_tcp_frame.c_str());
            }
            m_print_cb_warning = true;
        } else{
            m_keep_base_orientation = true;
            m_E_p_EC.setZero();
            m_quat_E_C.setIdentity();
        }
        m_last_msg = ros::Time::now();
    }

    template<class T>
    void SnsTrkVelocityController<T>::snsTrkTwistCmdCb(const hrr_msgs::SnsTrkTwistCmdPtr &msg) {
        tf::vectorMsgToEigen(msg->twist.linear, m_vel_p_des);
        tf::vectorMsgToEigen(msg->twist.angular, m_vel_rot_des);
        tf::pointMsgToEigen(msg->center, m_E_p_EC);
        m_quat_E_C.setIdentity();
        m_keep_base_orientation = true;
        m_last_msg = ros::Time::now();
    }

    template<class T>
    bool SnsTrkVelocityController<T>::init(T *hw, ros::NodeHandle &nh) {
        SnsTrkVelRosParams pars{nh};
        if (!pars.valid) {
            ROS_ERROR("[%s] Invalid parameters %s", m_log_name.c_str(), pars.print_params().c_str());
            return false;
        }
        try {
            m_hw_handle = hw->getHandle(pars.sns_trk_hw_name);
        } catch (...) {
            ROS_ERROR("[%s] failed to initialize controller:%s", m_log_name.c_str(), pars.print_params().c_str());
            return false;
        }
        ROS_DEBUG("[%s] parameters\nread:%s", m_log_name.c_str(), pars.print_params().c_str());
        m_ee_v_max = pars.ee_v_max;
        m_ee_omega_max = pars.ee_omega_max;
        if (pars.loop_rate > 0.0)
            m_dt = 1.0 / pars.loop_rate;
        ROS_DEBUG("[%s] set dt:=%.3f", m_log_name.c_str(), m_dt);
        m_debug = pars.debug;
        m_cmd_subs.push_back(nh.subscribe(pars.cmd_topic_name, 1, &SnsTrkVelocityController::twistCmdCb, this));
        m_cmd_subs.push_back(
                nh.subscribe(pars.sns_trk_cmd_topic_name, 1, &SnsTrkVelocityController::snsTrkTwistCmdCb, this));
        m_reference_frame = getReferenceFrame(m_hw_handle);
        m_tf_buffer.reset(new tf2_ros::Buffer());
        m_tf_listener.reset(new tf2_ros::TransformListener(*m_tf_buffer, true));
        std::shared_ptr<tf2_ros::TransformListener> tfListener_{};
        ROS_INFO("[%s] initialized controller in frame: %s", m_log_name.c_str(), m_reference_frame.c_str());
        return true;
    }


typedef SnsTrkVelocityController<hardware_interface::comau::SensorTrackingCommandInterface> STVelocityController;
typedef SnsTrkVelocityController<hardware_interface::comau::SensorTrackingPoseCommandInterface>
    STPoseVelocityController;
}  // namespace hrr_controllers
PLUGINLIB_EXPORT_CLASS(hrr_controllers::STVelocityController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(hrr_controllers::STPoseVelocityController, controller_interface::ControllerBase)
