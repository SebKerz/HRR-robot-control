#include <hrr_common/utils.h>

namespace hrr_cobot{


    double clip(const double &v, const double &v_min, const double &v_max){
        return std::max(v_min, std::min(v, v_max));
    }

    Eigen::Vector3d clip(const Eigen::Vector3d &v, const double &v_min, const double &v_max){
        Eigen::Vector3d out{v};
        for (auto i=0; i<3; i++){
            out(i) = std::max(v(i), std::min(v(i), v_max));
        }
        return out;
    }

    std::string getParamNs(const ros::NodeHandle &nh, const std::string &param_ns_param_name){
        std::string param_ns;
        nh.param<std::string>(param_ns_param_name, param_ns, param_ns);
        return param_ns;
    }

    std::tuple<ros::Duration, struct timespec> temporalDifference(const struct timespec &last_time) {
        struct timespec now{};
        clock_gettime(CLOCK_MONOTONIC, &now);
        return std::make_tuple(ros::Duration(double(now.tv_sec - last_time.tv_sec )+
                                             double (now.tv_nsec - last_time.tv_nsec) / BILLION), now);
    }

    ros::Time toRosTime(const struct timespec &ts){
        return ros::Time(double(ts.tv_sec),  double(ts.tv_nsec) / BILLION);
    }

    std::tuple<bool, std::vector<std::string>> getJointNames(
        const ros::NodeHandle &nh, const std::string &param_name, const std::string &name) {
        std::vector<std::string> joint_names{};
        auto error = !rosparam_shortcuts::get(name, nh, param_name, joint_names);
        if (error > 0) {
            return std::make_tuple(false, joint_names);
        }
        return std::make_tuple(true, joint_names);
    }

    std::tuple<bool, std::vector<hardware_interface::JointHandle>> getJointHandles(
            hardware_interface::EffortJointInterface *hw, const std::vector<std::string> &joint_names, const std::string &name
    ) {
        bool error = false;
        std::vector<hardware_interface::JointHandle> joint_handles{};
        for (auto &joint_name : joint_names) {
            try {
                joint_handles.push_back(hw->getHandle(joint_name));
                ROS_DEBUG_STREAM("[" << name << "] found joint " << joint_name);
            } catch (const hardware_interface::HardwareInterfaceException &e) {
                ROS_ERROR_STREAM("[" << name << "] could not find handle for " << joint_name << ": " << e.what());
                error = true;
            }
        }
        return std::make_tuple(error, joint_handles);
    };

    bool strEndsWith(const std::string s1, const std::string ending){
        return  s1.compare(s1.size() - ending.size(), ending.size(), ending) == 0;
    }


    std::string getBaseParam(const std::string &param_ns) {
        std::string base_param{param_ns};
        if ((!param_ns.empty()) and (param_ns != "/")) {
            base_param += "/";
        }
        return base_param;
    }
    
    std::string getBaseParam(ros::NodeHandle &nh, const std::string &param_ns) {
        std::string base_param{nh.getNamespace()};
        if ((!param_ns.empty())) {
            base_param += "/" + param_ns;
        }
        if (!(strEndsWith(base_param,"/"))){
             base_param += "/";
        }
        return base_param;
    }



}