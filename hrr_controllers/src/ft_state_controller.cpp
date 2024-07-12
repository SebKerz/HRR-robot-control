#include <hrr_controllers/ft_state_controller.h>
#include <pluginlib/class_list_macros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace hrr_controllers
{
    SensorCalibParams::SensorCalibParams(const ros::NodeHandle &nh)
    {
        valid = getParam<double>(nh, CalibrateSensor::m_log_name, "publish_rate", publish_rate);
        if (valid)
            valid = getParam<std::string>(nh, CalibrateSensor::m_log_name, "sensor_hw_name", sensor_hw_name);
        nh.param<std::string>("ft_com_topic_name", ft_com_topic_name, ft_com_topic_name);
        nh.param<std::string>("ft_bias_topic_name", ft_bias_topic_name, ft_bias_topic_name);
        nh.param<int32_t>("buffer_size", buf_size, buf_size);
    }
    std::string SensorCalibParams::print_params() const
    {
        std::stringstream ss{};
        ss << "\n\tsensor_hw_name:            \t" << sensor_hw_name
           << "\n\tft_com_topic_name:         \t" << ft_com_topic_name
           << "\n\tft_bias_topic_name:        \t" << ft_bias_topic_name
           << "\n\tbuffer size:               \t" << buf_size;
        return ss.str();
    }

    SensorCalibStateControllerParams::SensorCalibStateControllerParams(const ros::NodeHandle &nh) {
        valid = getParam<double>(nh, CalibrateSensor::m_log_name, "publish_rate", publish_rate);
        nh.param<std::string>("raw_topic", raw_topic, raw_topic);
        nh.param<std::string>("filtered_topic", filtered_topic, filtered_topic);
        nh.param<std::string>("no_bias_topic", no_bias_topic, no_bias_topic);
        nh.param<std::string>("compensated_topic", compensated_topic, compensated_topic);
        nh.param<bool>("publish_raw", publish_raw, publish_raw);
        nh.param<bool>("publish_filtered", publish_filtered, publish_filtered);
        nh.param<bool>("publish_bias_free", publish_bias_free, publish_bias_free);
        nh.param<bool>("publish_compensated", publish_compensated, publish_compensated);
        if (raw_topic.empty())
            publish_raw = false;
        if (filtered_topic.empty())
            publish_filtered = false;
        if (no_bias_topic.empty())
            publish_bias_free = false;
        if (compensated_topic.empty())
            publish_compensated = false;
    }

    std::string SensorCalibStateControllerParams::print_params() const {
        std::stringstream ss{};
        if (publish_raw)
            ss << "\n\traw topic:        \t" << raw_topic;
        if (publish_filtered)
            ss << "\n\tfiltered_topic:   \t" <<filtered_topic;
        if (publish_bias_free)
            ss << "\n\tno_bias_topic:    \t" << no_bias_topic;
        if (publish_compensated)
            ss << "\n\tcompensated_topic:\t" << compensated_topic;
        return ss.str();
    }

    void CalibrateSensor::update_params()
    {
        std::string param_ns{};
        if (m_nh.searchParam("tool_com/com", param_ns))
        {
            m_nh.param<double>(param_ns + "/x", m_tool_com[0], m_tool_com[0]);
            m_nh.param<double>(param_ns + "/y", m_tool_com[1], m_tool_com[1]);
            m_nh.param<double>(param_ns + "/z", m_tool_com[2], m_tool_com[2]);
        }
        if (m_nh.searchParam("tool_com/mass", param_ns))
        {
            m_nh.param<double>(param_ns, m_mass, m_mass);
        }
        if (m_nh.searchParam("sensor_bias", param_ns))
        {
            m_nh.param<double>(param_ns + "/force/x", m_bias.first[0], m_bias.first[0]);
            m_nh.param<double>(param_ns + "/force/y", m_bias.first[1], m_bias.first[1]);
            m_nh.param<double>(param_ns + "/force/z", m_bias.first[2], m_bias.first[2]);
            m_nh.param<double>(param_ns + "/torque/x", m_bias.second[0], m_bias.second[0]);
            m_nh.param<double>(param_ns + "/torque/y", m_bias.second[1], m_bias.second[1]);
            m_nh.param<double>(param_ns + "/torque/z", m_bias.second[2], m_bias.second[2]);
            m_initialized = true;
        }
    }

    void CalibrateSensor::rosInit(hardware_interface::ForceTorqueSensorInterface *hw, ros::NodeHandle &nh)
    {
        m_nh = nh;
        SensorCalibParams p{nh};
        m_ft_handle = hw->getHandle(p.sensor_hw_name);
        m_ros_subs.push_back(nh.subscribe(p.ft_bias_topic_name, 1, &CalibrateSensor::setSensorBiasCb, this));
        m_ros_subs.push_back(nh.subscribe(p.ft_com_topic_name, 1, &CalibrateSensor::setToolComCb, this));
        m_bfr.first.resize(p.buf_size, 3);
        m_bfr.second.resize(p.buf_size, 3);
        update_params();
    }

    void CalibrateSensor::processData()
    {
        auto f = m_ft_handle.getForce();
        auto t = m_ft_handle.getTorque();
        for (auto i = 0; i < 3; i++)
        {
            m_raw.first[i] = f[i];
            m_raw.second[i] = t[i];
            m_bfr.first(m_roll_buf_idx, i) = f[i];
            m_bfr.second(m_roll_buf_idx, i) = t[i];
        }
        m_roll_buf_idx += 1;
        if (m_buf_size < m_bfr.first.rows())
        {
            m_buf_size += 1;
        }
        if (m_roll_buf_idx >= m_bfr.first.rows())
        {
            m_roll_buf_idx -= m_bfr.first.rows();
        }
    }

    void CalibrateSensor::resetBuffer()
    {
        m_bfr.first.fill(0.0);
        m_bfr.second.fill(0.0);
        m_roll_buf_idx = 0;
        m_buf_size = 0;
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> CalibrateSensor::getWrench() const
    {
        return std::make_pair(m_raw.first - m_bias.first, m_raw.second - m_bias.second);
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> CalibrateSensor::getWrench(const Eigen::Affine3d &T_base_sensor) const
    {
        auto wrench = getWrench();
        auto g_S = T_base_sensor.rotation().transpose() * gravity_B;
        return std::make_pair(wrench.first - m_mass * g_S,
                              wrench.second - (T_base_sensor.rotation() * m_tool_com).cross(m_mass * g_S));
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> CalibrateSensor::getWrench(int &n_filter) const
    {
        n_filter = std::min(n_filter, m_buf_size);
        if (n_filter > 1)
        {
            Eigen::Vector3d f_filt{}, t_filt{};
            throw std::runtime_error("not implemented");
            return std::make_pair(f_filt, t_filt);
            //            for (auto i = 0; i < 3; i++) {
            //                f_filt[i] = m_bfr.first.block<n_filter, 1>(m_buf_size - n_filter, i).sum() / n_filter;
            //                t_filt[i] = m_bfr.second.block<n_filter, 1>(m_buf_size - n_filter, i).sum() / n_filter;
            //            };
        }
        return getWrench();
    }

    void CalibrateSensor::setToolComCb(const hrr_msgs::ToolComPtr &msg)
    {
        tf::vectorMsgToEigen(msg->center_of_mass, m_tool_com);
        m_mass = msg->mass;
    }

    void CalibrateSensor::setSensorBiasCb(const geometry_msgs::WrenchPtr &msg)
    {
        tf::vectorMsgToEigen(msg->force, m_bias.first);
        tf::vectorMsgToEigen(msg->torque, m_bias.second);
        m_initialized = true;
    }

    bool CalibratedFTStateController::init(hardware_interface::ForceTorqueSensorInterface *hw, ros::NodeHandle &nh)
    {
        SensorCalibStateControllerParams p{nh};
        if (!p.valid)
        {
            ROS_ERROR("[%s] Invalid parameters %s", m_log_name.c_str(), p.print_params().c_str());
            return false;
        }
        m_publish_rate = p.publish_rate;
        try
        {
            CalibrateSensor::rosInit(hw, nh);
        }
        catch (...)
        {
            ROS_ERROR("failed to initialize calibrated FT-state controller");
            return false;
        }

        if (p.publish_raw)
        {
            m_pub_idxs.insert_or_assign(ft_raw, m_ros_pubs.size());
            m_last_publish_times.push_back(ros::Time::now());
            m_ros_pubs.push_back(std::make_shared<RtWrenchPub>(nh, p.raw_topic, 100));
            m_active_pubs.insert(ft_raw);
        }
        if (p.publish_bias_free)
        {
            m_pub_idxs.insert_or_assign(ft_no_bias, m_ros_pubs.size());
            m_last_publish_times.push_back(ros::Time::now());
            m_ros_pubs.push_back(std::make_shared<RtWrenchPub>(nh, p.no_bias_topic, 100));
            m_active_pubs.insert(ft_no_bias);
        }
        if (p.publish_compensated)
        {
            m_pub_idxs.insert_or_assign(ft_tool_comp, m_ros_pubs.size());
            m_last_publish_times.push_back(ros::Time::now());
            m_ros_pubs.push_back(std::make_shared<RtWrenchPub>(nh, p.compensated_topic, 100));
            m_active_pubs.insert(ft_tool_comp);
        }
        if (p.publish_filtered)
        {
            m_pub_idxs.insert_or_assign(ft_filtered, m_ros_pubs.size());
            m_last_publish_times.push_back(ros::Time::now());
            m_ros_pubs.push_back(std::make_shared<RtWrenchPub>(nh, p.filtered_topic, 100));
            m_active_pubs.insert(ft_filtered);
        }
        for (auto pub : m_ros_pubs)
        {
            pub->msg_.header.frame_id = getSensorFrame();
        }
        ROS_DEBUG_NAMED(m_log_name, "[%s] started FT calibration state controller: %s", m_log_name.c_str(),
                        p.print_params().c_str());
        return true;
    }

    void CalibratedFTStateController::publish_helper(const int &id, const geometry_msgs::Wrench &wrench_msg,
                                                     const ros::Time &time)
    {
        try
        {
            if (m_last_publish_times[id] + ros::Duration(1 / m_publish_rate) < time)
            {
                if (m_ros_pubs[id]->trylock())
                {
                    m_ros_pubs[id]->msg_.header.stamp = time;
                    m_ros_pubs[id]->msg_.wrench = wrench_msg;
                    m_last_publish_times[id] += ros::Duration(1 / m_publish_rate);
                    m_ros_pubs[id]->unlockAndPublish();
                }
            }
            else
            {
            }
        }
        catch (...)
        {
            ROS_DEBUG_NAMED(m_log_name, "failed to publish data for publisher #%d", id + 1);
        }
    }

    void CalibratedFTStateController::update(const ros::Time &time, const ros::Duration &period)
    {
        CalibrateSensor::processData();
        if (m_publish_rate <= 0.0)
        {
            return;
        }
        if (m_active_pubs.find(ft_raw) != m_active_pubs.end())
        {
            publish_helper(m_pub_idxs[ft_raw], toMsg(getRaw()), time);
        }
        if (m_active_pubs.find(ft_no_bias) != m_active_pubs.end())
        {
            publish_helper(m_pub_idxs[ft_no_bias], toMsg(getWrench()), time);
        }
        if (m_active_pubs.find(ft_filtered) != m_active_pubs.end())
        {
            publish_helper(m_pub_idxs[ft_filtered], toMsg(getWrench(m_window_size)), time);
        }
        if (m_active_pubs.find(ft_tool_comp) != m_active_pubs.end())
        {
            publish_helper(m_pub_idxs[ft_tool_comp], toMsg(getWrench(m_cur_pose)), time);
        }
    }

    void CalibratedFTStateController::starting(const ros::Time &time)
    {
        CalibrateSensor::resetBuffer();
        for (auto &pub_t : m_last_publish_times)
        {
            pub_t = time;
        }
    }

} // namespace hrr_controllers
PLUGINLIB_EXPORT_CLASS(hrr_controllers::CalibratedFTStateController, controller_interface::ControllerBase)
