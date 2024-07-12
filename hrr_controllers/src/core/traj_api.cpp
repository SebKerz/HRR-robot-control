//
// Created by gabler on 26.01.21.
//

#include <hrr_controllers/core/traj_api.h>


namespace hrr_controllers {
    /** PRIVATE Functions **/

    /** PUBLIC FUNCTIONS **/

    bool setReset(GenericTrajHandle &handle){
        bool out{false};
        std::visit(overload{[&out](auto &h) {
            try {
                h.setReset(true);
                out = true;
            } catch (...) {
                ROS_ERROR("could not set reset flag for current hw-handle");
            }
        }}, handle);
        return out;
    }

    std::vector<std::array<double, 6>> getCurrentTrajectory(const GenericTrajHandle &handle){
        std::vector<std::array<double, 6>> out{};
        std::visit(overload{
                [&out](hardware_interface::comau::TrajectoryCommandHandle &h) {
                    try {
                        out = h.getTrajectory();
                    } catch (...) {
                        ROS_ERROR("could not get current trajectory from hw-handle");
                    }
                },
                [&out](hardware_interface::comau::CartesianTrajectoryCommandHandle &h) {
                    try {
                        out = h.getTrajectory();
                    } catch (...) {
                        ROS_ERROR("could not get current trajectory from hw-handle");
                    }
                },
                [&out](auto &h) {}
        }, handle);
        return out;
    }

    bool setTrajectory(GenericTrajHandle &handle, const std::vector<std::array<double, 6>> traj_cmd) {
        bool out{false};
        std::visit(overload{
                [&traj_cmd, &out](hardware_interface::comau::TrajectoryCommandHandle &h) {
                    try {
                        h.setTrajectory(traj_cmd);
                        out = true;
                    } catch (...) {
                        ROS_ERROR("could not get current trajectory from hw-handle");
                    }
                },
                [&traj_cmd, &out](hardware_interface::comau::CartesianTrajectoryCommandHandle &h) {
                    try {
                        h.setTrajectory(traj_cmd);
                        out = true;
                    } catch (...) {
                        ROS_ERROR("could not get current trajectory from hw-handle");
                    }
                },
                [](auto &h) {}
        }, handle);
        return out;
    }


} // namespace