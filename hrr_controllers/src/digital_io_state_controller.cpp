//
// Created by gabler on 26.01.21.
//

#include <hrr_controllers/digital_io_state_controller.h>
#include <pluginlib/class_list_macros.h>

namespace hrr_controllers
{

    bool DigitalIOStateController::init(hardware_interface::comau::DigitalIOStateInterface *hw,
                                        ros::NodeHandle &ctrl_nh)
    {
        m_nh = ctrl_nh;
        ctrl_nh.param<double>("publish_rate", m_publish_rate, m_publish_rate);
        if (m_publish_rate <= 0.0){
            ROS_ERROR("[%s] Invalid publish rate %.2f", m_log_name.c_str(), m_publish_rate);
            return false;
        }
        std::string din_name{"DIN"};
        std::string dout_name{"DOUT"};
        ctrl_nh.param<std::string>("din_name", din_name, din_name);
        ctrl_nh.param<std::string>("dout_name", dout_name, dout_name);
        try{
            m_io_handles.first = hw->getHandle(din_name);
        } catch (...){
                ROS_ERROR("[%s] failed to connect to input pin %s", m_log_name.c_str(), din_name.c_str());
                return false;
        }
        try{
            m_io_handles.second = hw->getHandle(dout_name);
        } catch (...){
                ROS_ERROR("[%s] failed to connect to output pin %s", m_log_name.c_str(), din_name.c_str());
                return false;
        }

        std::string din_topic_name{"io"};
        ctrl_nh.param<std::string>("din_topic_name", din_topic_name, din_topic_name);
        m_io_pub = std::make_shared<RtIOsPub>(ctrl_nh, din_topic_name, 100);
        return true;
    }
    
    std::vector<comau_msgs::Digital> convert(const std::vector<int8_t> &values){
        std::vector<comau_msgs::Digital> digitals{};
        digitals.resize(values.size());
        for (size_t i=0; i < values.size(); i++){
            comau_msgs::Digital current{};
            current.pin = i + 1;
            current.state = values[i] > 0;
            digitals[i] = std::move(current);
        }
        return digitals;
    }

    std::string pin_str(const std::vector<int8_t> &values, const std::string &init_str){
        std::stringstream pstr{};
        pstr << init_str << ":[";
        for (auto&pin: values){
            if (pin > 0){
                pstr << "1, ";
            } else{
                pstr << "0, ";
            }
        }
        pstr << "]";
        return pstr.str();
    }

    void DigitalIOStateController::update(const ros::Time &time, const ros::Duration &period)
    {
        if (m_prev_pub + ros::Duration(1.0 / m_publish_rate) < time){      
            auto din = m_io_handles.first.getValues();
            auto dout = m_io_handles.second.getValues();
            ROS_DEBUG_THROTTLE_NAMED(2.0, m_log_name, "[%s] read digital states:\n%s\n%s", m_log_name.c_str(), 
                                    (pin_str(din, "DIN")).c_str(),
                                    (pin_str(dout, "DOUT")).c_str());
            m_io_pub->msg_.digital_in_states.clear();
            m_io_pub->msg_.digital_in_states = convert(din);
            m_io_pub->msg_.digital_out_states.clear();
            m_io_pub->msg_.digital_out_states = convert(dout);
            if (m_io_pub->trylock()) {
                m_io_pub->unlockAndPublish();
                m_prev_pub += ros::Duration(1.0 / m_publish_rate);
            }
        }
    }


} // namespace hrr_controllers
PLUGINLIB_EXPORT_CLASS(hrr_controllers::DigitalIOStateController, controller_interface::ControllerBase);

