//
// Created by gabler on 26.01.21.
//

#ifndef HRR_CONTROLLERS_TRAJ_API_H
#define HRR_CONTROLLERS_TRAJ_API_H

// default
#include <iostream>
#include <chrono>
#include <ctime>

//ROS
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <comau_driver/helpers.h>

// hrr-controller utils
#include <hrr_common/utils.h>
#include <hrr_controllers/core/utils.h>


namespace hrr_controllers {

    [[nodiscard]] std::vector<std::array<double, 6>> getCurrentTrajectory(const GenericTrajHandle &handle);

    bool setTrajectory(GenericTrajHandle &handle, const std::vector<std::array<double, 6>> traj_cmd);

    bool setReset(GenericTrajHandle &handle);

} // namespace hrr_controllers
#endif //HRR_CONTROLLERS_SNS_TRK_BASE_CONTROLLER_H
