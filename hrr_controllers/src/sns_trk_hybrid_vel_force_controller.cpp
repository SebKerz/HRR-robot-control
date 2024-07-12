//
// Created by gabler on 27.01.21.
//

#include <hrr_controllers/sns_trk_hybrid_vel_force_controller.h>
#include <pluginlib/class_list_macros.h>

namespace hrr_controllers {

    Eigen::Matrix3d arr2Mat(const bool &x, const bool &y, const bool &z) {
        Eigen::Matrix3d out{Eigen::Matrix3d::Zero()};
        out(0, 0) = (x) ? 1.0 : 0.0;
        out(1, 1) = (y) ? 1.0 : 0.0;
        out(2, 2) = (z) ? 1.0 : 0.0;
        return out;
    }

    template<typename T, const int N>
    Eigen::Matrix<T, 3, 3> arr2Mat(const std::array<T, N> &arr, const unsigned int &start) {
        Eigen::Matrix<T, 3, 3> out{};
        out.setZero();
        assert (start + 3 <= N);
        for (auto i = 0; i < 3; i++) {
            out(i, i) = arr[i + start];
        }
        return out;
    }

    std::array<bool, 6> selectMsg2Arr(const std::vector<uint8_t> &S_msg,
                                      const std::array<bool, 6> &S_prev,
                                      const std::string &debug_str=""
                                      ){
        std::array<bool, 6> S {S_prev};
        std::transform(S_msg.cbegin(), S_msg.cend(), S.begin(), [](const uint8_t s) -> bool { return s > 0; });
        if (S_prev != S){
            std::stringstream str1{"\nS_prev:\t"}, str2{"\nS_new:\t"};
            for (auto i=0; i < 6; i++){
                str1 << S_prev[i] << ", ";
                str2 << S[i] << ", ";
            }
            ROS_DEBUG_NAMED(
                CompliantController<hardware_interface::comau::SensorTrackingPoseCommandInterface>::m_log_name,
                 "[%s] %s", CompliantController<hardware_interface::comau::SensorTrackingPoseCommandInterface>::m_log_name.c_str(),
                 (debug_str + str1.str() + str2.str()).c_str());
        }
        return S;
    }

    void publish_helper(RtWrenchPubPtr &pub_handle, const Eigen::Vector3d &f, const Eigen::Vector3d &m,
                        const ros::Time &time) {
        try {
            geometry_msgs::Wrench wrench_msg{};
            tf::vectorEigenToMsg(f, wrench_msg.force);
            tf::vectorEigenToMsg(m, wrench_msg.torque);
            if (pub_handle->trylock()) {
                pub_handle->msg_.header.stamp = time;
                pub_handle->msg_.wrench = wrench_msg;
                pub_handle->unlockAndPublish();
            }
        } catch (...) {
            ROS_DEBUG("failed to publish FT data");
        }
    }


    template<class T>
    void CompliantController<T>::update(const ros::Time &time, const ros::Duration &period) {
        if (m_last_write + ros::Duration(m_dt) > time) {
            ROS_DEBUG_NAMED(m_log_name, "next time step %.3f (t_prev=%.3f, dt = %.3f) > current %.3f",
                            (m_last_write + ros::Duration(m_dt)).toSec(),
                            m_last_write.toSec(), m_dt, time.toSec());
            return;
        }

        if (m_publish_ft_data) {
            publish_helper(m_ft_pubs[0], m_B_force_cur, m_B_torque_cur, time);
            publish_helper(m_ft_pubs[1], m_B_force_des - m_B_force_cur,
                           m_B_torque_des - m_B_torque_cur, time);
        }

        bool reset_controller{false}, disable_force_control{false};
        std::string warn_msg = "";
        if (m_last_ft_read + ros::Duration(m_dt_timeout) < time) {
            reset_controller = true;
            warn_msg += "did not receive new FT data\n";
        }
        if (m_last_msg + ros::Duration(m_dt_timeout) < time) {
            reset_controller = true;
            warn_msg += "timeout since last message\n";
        }
        if (reset_controller) {
            if (m_print_warning) {
                ROS_WARN("[%s] %s", m_log_name.c_str(), warn_msg.c_str());
                m_print_warning = false;
            }
            setZero();
            return;
        }

        // check if force-control is applicable
        warn_msg = "";
        if (m_B_force_cur.norm() + m_B_torque_cur.norm() == 0.0) {
            disable_force_control = true;
            warn_msg += "no wrench data obtained\n";
        }
        if (m_B_force_cur.norm() > m_F_max) {
            disable_force_control = true;
            warn_msg += "current force measurement norm |F|:" + std::to_string(m_B_force_cur.norm()) +
                        " exceeds maximum " + std::to_string(m_F_max) + "\n";
        }
        if (m_B_torque_cur.norm() > m_M_max) {
            disable_force_control = true;
            warn_msg += "current torque measurement norm |M|:" + std::to_string(m_B_torque_cur.norm()) +
                        " exceeds maximum " + std::to_string(m_M_max) + "\n";
        }
        if (m_print_FT_warnings && !warn_msg.empty()){
            ROS_WARN("[%s] %s", m_log_name.c_str(), warn_msg.c_str());
            m_S_force.fill(0.0);
        }


        Eigen::Vector3d B_v_cmd{}, B_rot_cmd{};
        Eigen::Quaterniond B_quat_E{};
        tf::quaternionMsgToEigen(getRotation(m_hw_handle), B_quat_E);
        Eigen::Matrix3d R_B_C{B_quat_E * m_quat_E_C};

        // calculate control signal
        if (m_keep_base_orientation) {
            Eigen::Vector3d u_f{arr2Mat(m_S_force[0], m_S_force[1], m_S_force[2]) *
                                (arr2Mat<double, 6>(m_K_p_force, 0) * (m_B_force_des - m_B_force_cur))};
            Eigen::Vector3d u_t{arr2Mat(m_S_force[3], m_S_force[4], m_S_force[5]) *
                                (arr2Mat<double, 6>(m_K_p_force, 3) * (m_B_torque_des - m_B_torque_cur))};
            B_v_cmd = arr2Mat(m_S_vel[0], m_S_vel[1], m_S_vel[2]) * m_vel_p_des + u_f;
            B_rot_cmd = arr2Mat(m_S_vel[3], m_S_vel[4], m_S_vel[5]) * m_vel_p_des + u_t;
        } else {
            Eigen::Vector3d u_f{R_B_C * arr2Mat(m_S_force[0], m_S_force[1], m_S_force[2]) *
                                R_B_C.transpose() *
                                (arr2Mat<double, 6>(m_K_p_force, 0) * (m_B_force_des - m_B_force_cur))};
            Eigen::Vector3d u_t{R_B_C * arr2Mat(m_S_force[3], m_S_force[4], m_S_force[5]) *
                                R_B_C.transpose() *
                                (arr2Mat<double, 6>(m_K_p_force, 3) * (m_B_torque_des - m_B_torque_cur))};
            B_v_cmd = R_B_C * arr2Mat(m_S_vel[0], m_S_vel[1], m_S_vel[2]) * m_vel_p_des + u_f;
            B_rot_cmd = R_B_C * arr2Mat(m_S_vel[3], m_S_vel[4], m_S_vel[5]) * m_vel_rot_des + u_t;
        }

        // transform command velocity according to current reference frame / center of orientation
        B_v_cmd += (B_quat_E.matrix() * m_E_p_EC).cross(R_B_C * m_vel_rot_des);

        // scale commands to limits
        hrr_cobot::clipScaleVector<double>(B_v_cmd,   m_ee_v_max);
        hrr_cobot::clipScaleVector<double>(B_rot_cmd, m_ee_omega_max);
        if ((std::any_of(m_S_force.cbegin(), m_S_force.cend(), [](bool s){return s;})) or
            (std::any_of(m_S_vel.cbegin(),   m_S_vel.cend(),   [](bool s){return s;}))) {
                if (B_v_cmd.norm() > 0.0){
                    ROS_DEBUG_STREAM_NAMED(
                        m_log_name, "B_v_cmd:\t" << (B_v_cmd * 1e3).transpose() << " [mm/s]");
                }
                if (B_rot_cmd.norm() > 0.0){
                    ROS_DEBUG_STREAM_NAMED(
                        m_log_name, "B_v_rot:\t" << (B_rot_cmd * 180.0 / 3.1415).transpose() << " [deg/s]");
                }
        }

        // write to hardware
        if (!setEigenPose(m_hw_handle, B_v_cmd * m_dt, hrr_cobot::zeroOrderDifference(B_rot_cmd, m_dt))) {
            ROS_WARN("failed to set pose to robot");
        }

        m_last_write += ros::Duration{m_dt};
        m_print_warning = true;
    }

    template<class T>
    void CompliantController<T>::starting(const ros::Time &time) {
        setZero();
        m_reference_frame = getReferenceFrame(m_hw_handle);
        m_tcp_frame = getTargetFrame(m_hw_handle);
        m_last_write = time;
        m_print_warning = true;
        m_ft_calibrated = false;
        m_print_cb_warning = true;
        m_print_S_warning = true;
        m_print_FT_warnings = true;
    }

    template<class T>
    void CompliantController<T>::stopping(const ros::Time &time) {
        setZero();
    }

    template<class T>
    void CompliantController<T>::setZero() {
        m_vel_p_des.setZero();
        m_vel_rot_des.setZero();
        m_B_force_des = m_B_force_cur;
        m_B_torque_des = m_B_torque_cur;
        m_prev_control_frame = "";
        m_E_p_EC.setZero();
        m_quat_E_C.setIdentity();
        m_S_vel.fill(false);
        m_S_force.fill(false);
        if (!setEigenPose(m_hw_handle, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity())) {
            ROS_WARN("failed to set pose to robot");
        }
    }

    template<class T>
    bool CompliantController<T>::setCalibratedFlagSrv(std_srvs::SetBool::Request &req,
                                                      std_srvs::SetBool::Response &res) {
        if (req.data != m_ft_calibrated) {
            m_ft_calibrated = req.data;
            res.success = true;
        }
        m_print_S_warning = !m_ft_calibrated;
        return true;
    }

    template<class T>
    void CompliantController<T>::setGainsCb(hrr_controllers::GainsConfig &gain_config, uint32_t level) {
        // force P-gains
        m_K_p_force.at(0) = hrr_cobot::clip(gain_config.K_P_fx, 0, m_K_p_f_max);
        m_K_p_force.at(1) = hrr_cobot::clip(gain_config.K_P_fy, 0, m_K_p_f_max);
        m_K_p_force.at(2) = hrr_cobot::clip(gain_config.K_P_fz, 0, m_K_p_f_max);

        m_K_p_force.at(3) = hrr_cobot::clip(gain_config.K_P_tx, 0, m_K_p_t_max);
        m_K_p_force.at(4) = hrr_cobot::clip(gain_config.K_P_ty, 0, m_K_p_t_max);
        m_K_p_force.at(5) = hrr_cobot::clip(gain_config.K_P_tz, 0, m_K_p_t_max);

        // force I-gains (currently not in use)
        m_K_i_force.at(0) = hrr_cobot::clip(gain_config.K_I_fx, 0, 1e-2 * m_K_p_f_max);
        m_K_i_force.at(1) = hrr_cobot::clip(gain_config.K_I_fy, 0, 1e-2 * m_K_p_f_max);
        m_K_i_force.at(2) = hrr_cobot::clip(gain_config.K_I_fz, 0, 1e-2 * m_K_p_f_max);

        m_K_i_force.at(3) = hrr_cobot::clip(gain_config.K_I_tx, 0, 1e-2 * m_K_p_t_max);
        m_K_i_force.at(4) = hrr_cobot::clip(gain_config.K_I_ty, 0, 1e-2 * m_K_p_t_max);
        m_K_i_force.at(5) = hrr_cobot::clip(gain_config.K_I_tz, 0, 1e-2 * m_K_p_t_max);
        ROS_DEBUG_STREAM_NAMED(m_log_name, "new control gain matrices:\n" <<
                                "K_p_F:\t[" << m_K_p_force[0] << ", " <<  m_K_p_force[1] << ", " <<  m_K_p_force[2] << ", "
                                            << m_K_p_force[3] << ", " <<  m_K_p_force[4] << ", " <<  m_K_p_force[5] << "] "
                                "\nK_i_F:\t[" << m_K_i_force[0] << ", " <<  m_K_i_force[1] << ", " <<  m_K_i_force[2] << ", "
                                              << m_K_i_force[3] << ", " <<  m_K_i_force[4] << ", " <<  m_K_i_force[5] << "]"
        );
    }

    template<class T>
    void CompliantController<T>::setReferenceFrame(const std::string &frame_id) {
        if (frame_id.empty()) {
            if (m_print_cb_warning) {
                ROS_WARN("[%s] empty frame_id received. Assuming default control: %s in %s",
                         m_log_name.c_str(),
                         m_tcp_frame.c_str(),
                         m_reference_frame.c_str());
                m_print_cb_warning = false;
            }
            m_E_p_EC.setZero();
            m_quat_E_C.setIdentity();
            m_keep_base_orientation = true;
        } else if ((frame_id != m_reference_frame) and (frame_id != m_tcp_frame)) {
            try {
                std::tie(m_E_p_EC, m_quat_E_C) = hrr_cobot::get_tf_helper(m_tf_buffer, m_tcp_frame, frame_id);
                m_keep_base_orientation = m_quat_E_C.isApprox(Eigen::Quaterniond::Identity());
                if (m_prev_control_frame != frame_id) {
                    m_prev_control_frame = frame_id;
                    if (m_keep_base_orientation) {
                        ROS_INFO_STREAM_NAMED(
                                m_log_name, "[" << m_log_name << "] received control frame (" << m_prev_control_frame
                                                << ") C_p_EC:= " << m_E_p_EC.transpose() << "without rotation!");
                    } else {
                        ROS_INFO_STREAM_NAMED(
                                m_log_name, "[" << m_log_name << "] received control frame (" << m_prev_control_frame
                                                << ") C_p_EC:= " << m_E_p_EC.transpose() << "R_E_C:="
                                                << m_quat_E_C.matrix());
                    }
                } else {
                    ROS_DEBUG_STREAM_NAMED(
                            m_log_name, "[" << m_log_name << "] received control frame (" << m_prev_control_frame
                                            << ") C_p_EC:= " << m_E_p_EC.transpose() << "R_E_C:="
                                            << m_quat_E_C.matrix());
                }
            } catch (tf2::TransformException &ex) {
                ROS_WARN("[%s] Could NOT get transformation from %s to %s",
                         m_log_name.c_str(), frame_id.c_str(), m_tcp_frame.c_str());
            }
            m_print_cb_warning = true;
        } else {
            m_keep_base_orientation = true;
            m_E_p_EC.setZero();
            m_quat_E_C.setIdentity();
        }
    }

    template<class T>
    void CompliantController<T>::controlSelectHelper(const std::vector<uint8_t> &S_force,
                                                     const std::vector<uint8_t> &S_vel) {
        if (m_ft_calibrated) {
           m_S_force = selectMsg2Arr(S_force, m_S_force, "S_F:");
        } else {
            if (m_print_S_warning){
                ROS_WARN("cannot enable force control while the controller assumes the FT-sensor to be not calibrated");
                m_print_S_warning = false;
            }
            m_S_force.fill(false);
        }
        m_S_vel = selectMsg2Arr(S_vel, m_S_force, "S_v:");
    }

    template<class T>
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> CompliantController<T>::wrench2base_frame(
            const std::string &reference_frame_id,
            const geometry_msgs::Wrench &msg) {
        Eigen::Vector3d f{}, t{}, p{};
        Eigen::Quaterniond q_B_FT{};
        tf::vectorMsgToEigen(msg.force, f);
        tf::vectorMsgToEigen(msg.torque, t);
        if ((reference_frame_id.empty()) or (reference_frame_id == m_reference_frame)) {
            return std::make_tuple(f, t);
        }
        std::tie(p, q_B_FT) = hrr_cobot::get_tf_helper(m_tf_buffer, reference_frame_id, m_reference_frame);
        auto R_B_FT = q_B_FT.matrix();
        return std::make_tuple(R_B_FT * f, R_B_FT * t);
    }

    template<class T>
    void CompliantController<T>::twistCmdCb(const geometry_msgs::TwistStampedConstPtr &msg) {
        tf::vectorMsgToEigen(msg->twist.linear, m_vel_p_des);
        tf::vectorMsgToEigen(msg->twist.angular, m_vel_rot_des);
        setReferenceFrame(msg->header.frame_id);
        m_last_msg = ros::Time::now();
    }

    template<class T>
    void CompliantController<T>::controlSelectCb(const hrr_msgs::HybridForceVelocityControlSelectConstPtr &msg) {
        controlSelectHelper(msg->S_force, msg->S_vel);
        m_last_msg = ros::Time::now();
    }

    template<class T>
    void CompliantController<T>::fullCmdCb(const hrr_msgs::HybridForceVelocityCmdStampedConstPtr &msg) {
        setReferenceFrame(msg->header.frame_id);
        controlSelectHelper(msg->cmd.select.S_force, msg->cmd.select.S_vel);
        tf::vectorMsgToEigen(msg->cmd.vel_cmd.linear, m_vel_p_des);
        tf::vectorMsgToEigen(msg->cmd.vel_cmd.angular, m_vel_rot_des);
        std::tie(m_B_force_des, m_B_torque_des) = wrench2base_frame(msg->header.frame_id, msg->cmd.F_cmd);
        m_last_msg = ros::Time::now();
    }

    template<class T>
    void CompliantController<T>::wrenchCmdCb(const geometry_msgs::WrenchStampedConstPtr &msg) {
        std::tie(m_B_force_des, m_B_torque_des) = wrench2base_frame(msg->header.frame_id, msg->wrench);
        m_last_msg = ros::Time::now();
    }

    template<class T>
    void CompliantController<T>::wrenchReadCb(const geometry_msgs::WrenchStampedConstPtr &msg) {
        std::tie(m_B_force_cur, m_B_torque_cur) = wrench2base_frame(msg->header.frame_id, msg->wrench);
        m_last_ft_read = ros::Time::now();
    }

    template<class T>
    bool CompliantController<T>::init(T *hw, ros::NodeHandle &nh) {
        CompliantControllerParams pars{nh};
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
        ROS_DEBUG("[%s] parameters\nread:%s", m_log_name.c_str(), pars.print_compliant_params().c_str());
        m_ee_v_max = pars.ee_v_max;
        m_ee_omega_max = pars.ee_omega_max;
        m_F_max = pars.f_max;
        m_M_max = pars.m_max;
        m_publish_ft_data = pars.publish_ft;
        if (pars.loop_rate > 0.0)
            m_dt = 1.0 / pars.loop_rate;
        ROS_DEBUG("[%s] set dt:=%.3f", m_log_name.c_str(), m_dt);
        m_debug = pars.debug;
        m_ros_subs.push_back(nh.subscribe(pars.wrench_cur_topic_name, 1,
                                          &CompliantController::wrenchReadCb, this));
        m_ros_subs.push_back(nh.subscribe(pars.wrench_cmd_topic_name, 1,
                                          &CompliantController::wrenchCmdCb, this));
        m_ros_subs.push_back(nh.subscribe(pars.cmd_topic_name, 1,
                                          &CompliantController::twistCmdCb, this));
        m_ros_subs.push_back(nh.subscribe(pars.control_select_topic_name, 1,
                                          &CompliantController::controlSelectCb, this));
        m_ros_subs.push_back(nh.subscribe(pars.full_cmd_topic_name, 1,
                                          &CompliantController::fullCmdCb, this));
        m_ros_srvs.push_back(nh.advertiseService(pars.set_calibrated_srv_name,
                                                 &CompliantController::setCalibratedFlagSrv, this));
        m_dyn_rcf_cb = boost::bind(&CompliantController::setGainsCb, this, _1, _2);
        m_dyn_rfc_server.setCallback(m_dyn_rcf_cb);
        m_reference_frame = getReferenceFrame(m_hw_handle);
        if (m_publish_ft_data) {
            m_ft_pubs.push_back(std::make_shared<RtWrenchPub>(nh, "B_ft_env", 100));
            m_ft_pubs.push_back(std::make_shared<RtWrenchPub>(nh, "B_ft_err", 100));
            for (auto &ft_pub: m_ft_pubs) {
                ft_pub->msg_.header.frame_id = m_reference_frame;
            }
        }
        m_tf_buffer.reset(new tf2_ros::Buffer());
        m_tf_listener.reset(new tf2_ros::TransformListener(*m_tf_buffer, true));
        std::shared_ptr<tf2_ros::TransformListener> tfListener_{};
        ROS_INFO("[%s] initialized controller in frame: %s", m_log_name.c_str(), m_reference_frame.c_str());
        return true;
    }

    typedef CompliantController<hardware_interface::comau::SensorTrackingCommandInterface> STCompliantController;
    typedef CompliantController<hardware_interface::comau::SensorTrackingPoseCommandInterface> STCompliantPoseController;
}  // namespace hrr_controllers
PLUGINLIB_EXPORT_CLASS(hrr_controllers::STCompliantController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(hrr_controllers::STCompliantPoseController, controller_interface::ControllerBase)