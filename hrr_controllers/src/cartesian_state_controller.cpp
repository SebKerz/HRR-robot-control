//
// Created by gabler on 26.01.21.
//

#include <hrr_controllers/cartesian_state_controller.h>
#include <pluginlib/class_list_macros.h>

namespace hrr_controllers
{
    CStateRosParams::CStateRosParams(const ros::NodeHandle &nh)
    {
        valid = getParam<double>(nh, CartesianStateController::m_log_name, "publish_rate", publish_rate);
        if (valid)
            valid = getParam<std::string>(nh, CartesianStateController::m_log_name, "hw_name", hw_name);
        nh.param<std::string>("raw_topic_name", raw_topic_name, raw_topic_name);
        nh.param<std::string>("tf_topic_name", tf_topic_name, tf_topic_name);
        nh.param<std::string>("sns_tcp_frame_name", sns_tcp_frame, sns_tcp_frame);
        
        nh.param<bool>("publish_tf", pub_tf, pub_tf);
        nh.param<bool>("publish_raw", pub_raw, pub_raw);
    }

    std::string CStateRosParams::print_params() const
    {
        std::stringstream ss{};
        ss << "\n\traw_topic_name:    \t" << raw_topic_name
           << "\n\ttf_topic_name:     \t" << tf_topic_name
           << "\n\thw_name:           \t" << hw_name
           << "\n\tsns_tcp_frame_name:\t" << sns_tcp_frame
           << "\n\tpublish_rate[1/s]: \t" << publish_rate
           << "\n\tpublish tf         \t" << pub_tf
           << "\n\tpublish raw        \t" << pub_raw;
        return ss.str();
    }

    comau_msgs::CartesianPoseStamped fromArray(const std::array<double, 6> &data)
    {
        comau_msgs::CartesianPoseStamped comau_raw{};
        comau_raw.header.stamp = ros::Time::now();
        comau_raw.x = data[0];
        comau_raw.y = data[1];
        comau_raw.z = data[2];
        comau_raw.roll = data[3];
        comau_raw.pitch = data[4];
        comau_raw.yaw = data[5];
        return comau_raw;
    }

    bool CartesianStateController::init(hardware_interface::comau::CartesianStateInterface *hw, ros::NodeHandle &nh,
                                        ros::NodeHandle &ctrl_nh)
    {
        m_nh = ctrl_nh;
        CStateRosParams pars{m_nh};
        if (!pars.valid)
        {
            ROS_ERROR("[%s] Invalid parameters %s", m_log_name.c_str(), pars.print_params().c_str());
            return false;
        }
        try
        {
            m_hw_handle = hw->getHandle(pars.hw_name);
        }
        catch (...)
        {
            ROS_ERROR("[%s] Failed to connect to hardware %s:\nparameters read %s", m_log_name.c_str(), pars.hw_name.c_str(),
                      pars.print_params().c_str());
            return false;
        }
        ROS_DEBUG("[%s] Connected to hardware %s", m_log_name.c_str(), pars.hw_name.c_str());
        m_comau_pose_pub = std::make_shared<RtCartPosePub>(m_nh, pars.raw_topic_name, 100);
        m_comau_pose_pub->msg_.header.frame_id = m_hw_handle.getBaseFrame();
        ROS_DEBUG("[%s] Adding raw Cart-Data publisher", m_log_name.c_str());
        m_tf_pose_pub = std::make_shared<RtTfPub>(m_nh, "/tf", 100);
        ROS_DEBUG("[%s] Adding tf broadcaster", m_log_name.c_str());
        m_tcp_frame = pars.sns_tcp_frame;
        return update_params();
    }

    void CartesianStateController::update(const ros::Time &time, const ros::Duration &period)
    {
        if (m_publish_rate > 0.0)
        {
            if (m_pub_tf_data.first && m_pub_tf_data.second + ros::Duration(1.0 / m_publish_rate) < time)
            {
                std::vector<geometry_msgs::TransformStamped> transforms {2};
                // raw data
                transforms.at(0).header.frame_id = m_hw_handle.getBaseFrame();
                transforms.at(0).child_frame_id = m_hw_handle.getTargetFrame();
                transforms.at(0).transform.translation = m_hw_handle.getTranslation();
                transforms.at(0).transform.rotation = m_hw_handle.getRotation();
                // set SNS-TCP
                transforms.at(1).header.frame_id = m_hw_handle.getTargetFrame();
                transforms.at(1).child_frame_id = m_tcp_frame;
                tf::vectorEigenToMsg(m_tcp_offset, transforms.at(1).transform.translation);
                tf::quaternionEigenToMsg(m_tcp_rotation, transforms.at(1).transform.rotation);
                for (auto &tf: transforms)
                    tf.header.stamp = ros::Time::now();
                if (m_tf_pose_pub->trylock()) {
                    m_tf_pose_pub->msg_.transforms.clear();
                    m_tf_pose_pub->msg_.transforms = std::move(transforms);
                    m_tf_pose_pub->unlockAndPublish();
                }
            }
            if (m_pub_pose_data.first && m_pub_pose_data.second + ros::Duration(1.0 / m_publish_rate) < time)
            {
                auto pose = fromArray(m_hw_handle.getPoseRaw());
                if (m_comau_pose_pub->trylock())
                {
                    m_comau_pose_pub->msg_ = pose;
                    m_comau_pose_pub->unlockAndPublish();
                    m_pub_pose_data.second += ros::Duration(1.0 / m_publish_rate);
                }
            }
        }
    }

    void CartesianStateController::starting(const ros::Time &time)
    {
        update_params();
        m_pub_tf_data.second = time;
        m_pub_pose_data.second = time;
    }

    bool CartesianStateController::update_params()
    {
        CStateRosParams pars{m_nh};
        m_publish_rate = pars.publish_rate;
        m_pub_pose_data.first = pars.pub_raw;
        m_pub_tf_data.first = pars.pub_tf;
        ROS_DEBUG("[%s] parameters:\nread:%s", m_log_name.c_str(), pars.print_params().c_str());
        return true;
    }

} // namespace hrr_controllers
PLUGINLIB_EXPORT_CLASS(hrr_controllers::CartesianStateController, controller_interface::ControllerBase);
