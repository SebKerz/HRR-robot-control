//
// Created by gabler on 13.01.21.
//

#ifndef HRR_COMMON_UTILS_H
#define HRR_COMMON_UTILS_H

// ros includes
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <hardware_interface/joint_command_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometry_msgs/TransformStamped.h>


namespace hrr_cobot {
    static const double BILLION = 1000000000.0;

    /**
     * @brief clip a double value to a min/max interval
     * 
     * @param v     valueto be clipped
     * @param v_min min value
     * @param v_max max value
     * @return double clipped value
     */
    double clip(const double &v, const double &v_min, const double &v_max);

    /**
     * @brief clip a 3d- double vector to a min and max interval per element
     * 
     * @param v     vector to be clipped
     * @param v_min min value
     * @param v_max max value
     * @return Eigen::Vector3d clipped vector
     */
    Eigen::Vector3d clip(const Eigen::Vector3d &v, const double &v_min, const double &v_max);

    /**
     * scale and clip vector
     * @tparam T type of vector
     * @param vec vector
     * @param max scalar maximum to scale norm or clip single components
     */
    template <typename T>
    void clipScaleVector(Eigen::Matrix<T, 3, 1> vec, const T& max){
        if (vec.norm() > max and vec.norm() > 1e-7){
            vec *= (max) / vec.norm();
        }
        vec = clip(vec, -max, max);
    }




    std::string getParamNs(const ros::NodeHandle &nh, const std::string &param_ns_param_name="");


    std::tuple<ros::Duration, struct timespec> temporalDifference(const struct timespec &last_time);


    ros::Time toRosTime(const struct timespec &ts);


    std::tuple<bool, std::vector<std::string>> getJointNames(const ros::NodeHandle &nh, const std::string &param_name,
            const std::string &name = ros::this_node::getName());


    std::tuple<bool, std::vector<hardware_interface::JointHandle>> getJointHandles(
            hardware_interface::EffortJointInterface *hw,
            const std::vector<std::string> &joint_names,
            const std::string &name = ros::this_node::getName());

    /**
     * @brief Get the Base Param object
     * 
     * @param param_ns 
     * @return std::string 
     */
    std::string getBaseParam(const std::string &param_ns = "");

    /**
     * @brief Get the Base Param object 
     * 
     * @param nh 
     * @param param_ns 
     * @return std::string 
     */
    std::string getBaseParam(ros::NodeHandle &nh, const std::string &param_ns);

    /**
     * @brief get transformation from TF module
     * buffer	        pointer to current tf2 buffer
     * target_frame	The frame to which data should be transformed
     * source_frame	The frame where the data originated 
    */
    inline std::tuple<bool, geometry_msgs::TransformStamped> getTransformation(const std::shared_ptr<tf2_ros::Buffer>& buffer,
                                                      const std::string & target_frame,
		                                      const std::string & source_frame){
        try{
                return std::make_tuple(true, buffer->lookupTransform(source_frame, target_frame, ros::Time(0)));
        } catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                return std::make_tuple(false, geometry_msgs::TransformStamped{});
        }
    }
    
    inline std::tuple<bool, Eigen::Affine3d> getEigenTransformation(const std::shared_ptr<tf2_ros::Buffer>& buffer,
                                                      const std::string & target_frame, const std::string & source_frame){
        auto[valid, T] = getTransformation(buffer, target_frame, source_frame);
        if (valid)
                return std::make_tuple(true, tf2::transformToEigen(T));
        else{
                return std::make_tuple(false, Eigen::Affine3d{});
        }
   }
}

#endif //COMAU_CONTROLLERS_PARAMS_H
