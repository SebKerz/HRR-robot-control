#include <hrr_cobot_robot/hrr_cobot_hw.h>


namespace hrr_cobot {

    struct SensorParams{
        std::string ns{"ft_sensor"};
        std::string type{"jr3_hw/JR3Sensor"};
        std::string joint_name{"jr3msr_joint"};

        explicit SensorParams(const ros::NodeHandle &nh_local){
            std::string info{};
            nh_local.param<std::string>("ft_sensor_ns", ns, ns);
            nh_local.param<std::string>(ns + "/hw_plugin", type, type);
            nh_local.param<std::string>(ns + "/sensor_joint_name", joint_name, joint_name);
        }
    };

    bool HrrCobotHW::init(ros::NodeHandle &nh, ros::NodeHandle &nh_local) {
        bool out{hardware_interface::comau::ComauRobotHW::init(nh, nh_local)};
        // init FT-sensor
        ROS_DEBUG("[%s] initialized sensor-loader ", m_log_name.c_str());
        SensorParams params{nh_local} ;
        pluginlib::ClassLoader<hardware_interface::ForceTorquePlugin> sensor_loader{"hardware_interface", "hardware_interface::ForceTorquePlugin"};
        try{
            m_ft_sensor.reset(sensor_loader.createUnmanagedInstance(params.type));
            m_ft_sensor->init(nh_local, this, params.ns);
            if (!m_ft_sensor->isRegistered()){
                ROS_WARN("[%s] failed to register FT-sensor!", m_log_name.c_str());
            } else{
                auto* joint_interface = this->get<hardware_interface::JointStateInterface>();
                joint_interface->registerHandle(m_ft_sensor->asJointStateHandle(params.joint_name));
            }
        } catch (pluginlib::LibraryLoadException &e){
            ROS_ERROR_STREAM("[" << m_log_name <<"] failed to load FT-sensor\nread parameters from "
                             << params.ns <<" encountered error: "
                             << e.what());
            return false;
        }
        return out;
    }

    void HrrCobotHW::read(const ros::Time &time, const ros::Duration &period){
        hardware_interface::comau::ComauRobotHW::read(time, period);
        if (m_ft_sensor){
            m_ft_sensor -> read();
        }
    }

    bool HrrCobotHW::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                     const std::list<hardware_interface::ControllerInfo> &stop_list) {
        if (!hardware_interface::comau::ComauRobotHW::prepareSwitch(start_list, stop_list)){
            std::stringstream ss{"blocked combination: "};
            for (auto &ctrl_info: start_list){
                ss << "name:  "<< ctrl_info.name << "of type: " << ctrl_info.type << ", ";
            }
            ROS_WARN("not implemented check for %s", ss.str().c_str());
            return false;
        }
        return true;
    }



}//namespace
