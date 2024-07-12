//
// Created by gabler on 17.02.21.
//

#ifndef HRR_CONTROLLERS_PARAMS_H
#define HRR_CONTROLLERS_PARAMS_H
#include <array>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace hrr_controllers{

    inline void raiseParameterMissing(const std::string& name, const std::string& param_name){
        ROS_ERROR("[%s] Missing parameter %s, expect undefined behavior!", name.c_str(), param_name.c_str());
    }

    // inline bool getParam(const ros::NodeHandle& nh, const std::string& name, double &value, const std::string param_name="publish_rate"){
    //     if (!rosparam_shortcuts::get(name, nh, param_name, value)){
    //         raiseParameterMissing(name, param_name);
    //         return false;
    //     }
    //     return true;
    // }

    template <typename T>
    bool getParam(const ros::NodeHandle& nh, const std::string& name, const std::string param_name, T &value){
        if (!rosparam_shortcuts::get(name, nh, param_name, value)){
            raiseParameterMissing(name, param_name);
            return false;
        }
        return true;
    }




}
#endif //HRR_CONTROLLERS_PARAMS_H
