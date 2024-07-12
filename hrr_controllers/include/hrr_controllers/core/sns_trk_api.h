//
// Created by gabler on 26.01.21.
//
#ifndef HRR_CONTROLLERS_SNS_TRK_API_H
#define HRR_CONTROLLERS_SNS_TRK_API_H

// default
#include <iostream>
#include <chrono>
#include <ctime>
#include <array>
//ROS
#include <controller_interface/controller.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>

// hrr-controller
#include <hrr_common/conversions.h>
#include <hrr_common/utils.h>
#include <hrr_controllers/core/utils.h>


namespace hrr_controllers {


    /**
     * !NOTE: this function is deprecated!
     * Write translation and rotation command to robot hardware*
     * @param handle generic hardware handle
     * @param p_cmd desired position in base_frame
     * @param rot_cmd desired Rotation relative to the base frame
     * @return true if successful
     * @return
     */
    bool setEigenPose(GenericSnsTrkHandle &handle, const Eigen::Vector3d &p_cmd, const Eigen::Quaterniond &rot_cmd);


    geometry_msgs::Quaternion getRotation(GenericSnsTrkHandle &handle);


    geometry_msgs::Vector3 getTranslation(GenericSnsTrkHandle &handle);


    std::tuple<geometry_msgs::Vector3, geometry_msgs::Quaternion> getPose(GenericSnsTrkHandle &handle);


    /**
     * @brief Get the Base Frame from hardware handle 
     * 
     * @param handle Cartesian Sensor Tracking interface
     * @return std::string base frame id (e.g. base-link)
     */
    std::string getBaseFrame(GenericSnsTrkHandle &handle);

    /**
     * @brief Get the Target Frame from hardware handle 
     *
     * @param handle Cartesian Sensor Tracking interface
     * @return std::string target frame (tcp) id
     */
    std::string getTargetFrame(GenericSnsTrkHandle &handle);

    /**
     * @brief Get the Reference Frame for the sensor tracking controller
     * 
     * @param handle Cartesian Sensor Tracking interface
     * @return std::string reference frame (world)
     */
    std::string getReferenceFrame(GenericSnsTrkHandle &handle);


    bool isRelative(GenericSnsTrkHandle &handle);


    bool isAbsolute(GenericSnsTrkHandle &handle);


    std::tuple<bool, geometry_msgs::Point, geometry_msgs::Quaternion> getPoseCmd(GenericSnsTrkHandle &handle);


    std::tuple<bool, std::array<double, 6>> getPoseArrayCmd(GenericSnsTrkHandle &handle);




} // namespace hrr_controllers
#endif //HRR_CONTROLLERS_SNS_TRK_API_H
