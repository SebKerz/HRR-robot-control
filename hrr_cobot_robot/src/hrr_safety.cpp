#include <comau_driver/helpers.h>
#include <hrr_cobot_robot/hrr_safety.h>



namespace hrr_cobot{

    using comau_driver::ControlMode;

    struct CoreParameters {        
        std::string dead_man_topic_name{"dead_man_switch"};
        std::string trigger_tool_srv_name;
        std::string dead_man_srv_name{"dead_man_state"};
        ros::Duration dead_man_timeout{0.2};

      CoreParameters(std::string &info, const std::string p_n=""){
            std::stringstream tmp{""};
            ros::NodeHandle nh{"~"};
            std::string base_param{};   
            double tmp_timeout{dead_man_timeout.toSec()};
            nh.param<double>(base_param + "dead_man_timeout", tmp_timeout, tmp_timeout);
            dead_man_timeout = ros::Duration{tmp_timeout};
            nh.param<std::string>(base_param + "pneumatic_tools_service_name", trigger_tool_srv_name, trigger_tool_srv_name);
            nh.param<std::string>(base_param + "dead_man_topic_name", dead_man_topic_name, dead_man_topic_name);
            nh.param<std::string>(base_param + "dead_man_srv_name", dead_man_srv_name, dead_man_srv_name);
            tmp << "\n\tdead_man_timeout: [s] \t" << dead_man_timeout
                << "\n\ttrigger_tool_srv_name:\t" << trigger_tool_srv_name
                << "\n\tdead-man topic:       \t" << dead_man_topic_name
                << "\n\tdead_man service:     \t" << dead_man_srv_name;
            info += tmp.str();
        }
    };


    bool SafetyWatchDog::init_ros(ros::NodeHandle &nh, const std::string &param_ns) {
        std::string info;        
        m_param_ns = hrr_cobot::getBaseParam(nh, "");
        CoreParameters params{info, m_param_ns};
        ROS_INFO("[%s] parameters:\nread @%s:%s", m_log_name.c_str(), m_param_ns.c_str(), info.c_str());
        m_dead_man_timeout = params.dead_man_timeout;
        m_dead_man_signal_sub = nh.subscribe(params.dead_man_topic_name, 1, &SafetyWatchDog::dead_man_cb, this);
        m_dead_man_state_srv = nh.advertiseService(params.dead_man_srv_name, &SafetyWatchDog::dead_man_state_srv, this);
        if (!params.trigger_tool_srv_name.empty()){
            auto srv_handle = nh.serviceClient<std_srvs::Trigger>(params.trigger_tool_srv_name);
            m_trigger_tool_srv.reset(&srv_handle);
        }
        return true;
    }
        
    void SafetyWatchDog::dead_man_cb(const hrr_msgs::DeadMan &msg) {
        struct timespec now{};
        clock_gettime(CLOCK_MONOTONIC, &now);        
        m_dead_man = msg.alive;
        if (msg.alive)
            m_last_dead_man = now;
    }



    bool SafetyWatchDog::dead_man_state_srv(hrr_msgs::GetDeadManState::Request &req,
                                            hrr_msgs::GetDeadManState::Response &res) {
        res.alive = m_alive;
        res.latest_feedback = toRosTime(m_last_dead_man);
        return true;
    }


    bool SafetyWatchDog::update(std::shared_ptr<HrrCobotHW> &hrr_cobot_hw, 
                                std::shared_ptr<controller_manager::ControllerManager> &controller_manager){
        auto [dt, d] = temporalDifference(m_last_dead_man);
        m_alive = (dt < m_dead_man_timeout) && m_dead_man;            
        if (m_alive)
            return true;
        bool set_emergency{false};
        for (const auto& [black_list_ctrl_mode, controller_names]: m_black_list_controllers){

            for (const auto& c_name: controller_names){
                try{
                    controller_manager->unloadController(c_name);
                    ROS_DEBUG("[%s] unloaded black-listed controller %s", m_log_name.c_str(), c_name.c_str());
                }
                catch(...){
                    ROS_ERROR("[%s] failed to unload controller %s", m_log_name.c_str(), c_name.c_str());
                }
                set_emergency = true;
            }
        }
        if (set_emergency){
            ROS_ERROR("[%s] locking robot has been removed and replaced by controller unloading.", m_log_name.c_str());
            if (m_trigger_tool_srv){
                std_srvs::Trigger srv{};
                m_trigger_tool_srv->call(srv);
                if (srv.response.success){
                    ROS_INFO("[%s] disabled tool" , m_log_name.c_str());
                } else {
                    ROS_ERROR("[%s] failed to disable tool" , m_log_name.c_str());
                }
            }
            ROS_ERROR("[%s] Safety conditions violated, locking Robot", m_log_name.c_str()); 
        }
        return true;
    }


}//namespace