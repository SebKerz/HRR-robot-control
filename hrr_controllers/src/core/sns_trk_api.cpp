//
// Created by gabler on 26.01.21.
//

#include <hrr_controllers/core/sns_trk_api.h>

namespace hrr_controllers {
    /** PRIVATE Functions **/

    /** PUBLIC FUNCTIONS **/
    bool
    setPose(GenericSnsTrkHandle &handle, const geometry_msgs::Point &p_cmd, const geometry_msgs::Quaternion &rot_cmd) {
        bool written{false};
        std::visit(overload{
                [&p_cmd, &rot_cmd, &written](hardware_interface::comau::SensorTrackingPoseCommandHandle &h) {
                    h.setCommand(p_cmd, rot_cmd);
                    written = true;
                },
                // todo: add
                //  [](hardware_interface::comau::SensorTrackingCommandHandle &h){},
                [](auto &h) {}
        }, handle);
        return written;
    }


    bool setArrayPose(GenericSnsTrkHandle &handle, const std::array<double, 6> &pose_py_cmd) {
        bool written{false};
        std::visit(overload{
                [&pose_py_cmd, &written](hardware_interface::comau::SensorTrackingCommandHandle &h) {
                    h.setCommand(pose_py_cmd);
                    written = true;
                },
                // todo: add
                //  [](hardware_interface::comau::SensorTrackingPoseCommandHandle &h){}
                [](auto &h) {}
        }, handle);
        return written;
    }


    bool setEigenPose(GenericSnsTrkHandle &handle, const Eigen::Vector3d &p_cmd, const Eigen::Quaterniond &rot_cmd) {
        bool written{false};
        geometry_msgs::Point p;
        geometry_msgs::Quaternion q;
        tf::pointEigenToMsg(p_cmd, p);
        tf::quaternionEigenToMsg(rot_cmd, q);
        std::visit(overload{
                [&p, &q, &written](hardware_interface::comau::SensorTrackingPoseCommandHandle &h) {
                    h.setCommand(p, q);
                    written = true;
                },
                [&p, &q, &written](hardware_interface::comau::SensorTrackingCommandHandle &h) {
                    h.setCommand(hrr_cobot::poseToArray(p, q));
                    written = true;
                },
                [](auto &h) {}
        }, handle);
        return written;
    }


    Rotation getRotation(GenericSnsTrkHandle &handle) {
        Rotation q;
        std::visit(overload{
                [&q](auto &h) { q = h.getRotation(); }
        }, handle);
        return q;
    }


    Translation getTranslation(GenericSnsTrkHandle &handle) {
        Translation p;
        std::visit(overload{
                [&p](auto &h) { p = h.getTranslation(); }
        }, handle);
        return p;
    }


    std::tuple<Translation, Rotation> getPose(GenericSnsTrkHandle &handle) {
        Translation p;
        Rotation q;
        std::visit(overload{
                [&p, &q](auto &h) {
                    p = h.getTranslation();
                    q = h.getRotation();
                }
        }, handle);
        return std::make_tuple(p, q);
    }

    std::tuple<bool, geometry_msgs::Point, geometry_msgs::Quaternion> getPoseCmd(GenericSnsTrkHandle &handle) {
        bool read{false};
        geometry_msgs::Point p;
        geometry_msgs::Quaternion q;
        std::visit(overload{
                [&p, &q, &read](hardware_interface::comau::SensorTrackingPoseCommandHandle &h) {
                    std::tie(p, q) = h.getCommand();
                    read = true;
                },
                [](auto &h) {}
        }, handle);
        return std::make_tuple(read, p, q);
    }

    std::tuple<bool, std::array<double, 6>> getPoseArrayCmd(GenericSnsTrkHandle &handle) {
        bool read{false};
        std::array<double, 6> cmd{0.};
        std::visit(overload{
                [&read, &cmd](hardware_interface::comau::SensorTrackingCommandHandle &h) {
                    cmd = h.getCommand();
                    read = true;
                },
                [](auto &h) {}
        }, handle);
        return std::make_tuple(read, cmd);
    }

    std::string getBaseFrame(GenericSnsTrkHandle &handle) {
        std::string base_frame;
        std::visit(overload{
                [&base_frame](auto &h) { base_frame = h.getBaseFrame(); }
        }, handle);
        return base_frame;
    }

    std::string getTargetFrame(GenericSnsTrkHandle &handle) {
        std::string tg_frame;
        std::visit(overload{
                [&tg_frame](auto &h) { tg_frame = h.getTargetFrame(); }
        }, handle);
        return tg_frame;
    }

    std::string getUFrame(GenericSnsTrkHandle &handle) {
        std::string tg_frame;
        std::visit(overload{
                [&tg_frame](auto &h) { tg_frame = h.getUFrame(); }
        }, handle);
        return tg_frame;
    }

    std::string getWeaveFrame(GenericSnsTrkHandle &handle) {
        std::string tg_frame;
        std::visit(overload{
                [&tg_frame](auto &h) { tg_frame = h.getWeaveFrame(); }
        }, handle);
        return tg_frame;
    }

    std::string getReferenceFrame(const SensorTrackParams &params, GenericSnsTrkHandle &handle) {
        switch (params.sensor_type) {
            case 5:
            case 9:
                // TOOL_REL = 5, TOOL_ABS = 9
                return getTargetFrame(handle);
            case 6:
            case 10:
                // UFRAME_REL = 6, UFRAME_ABS = 10
                return getUFrame(handle);
            case 7:
            case 11:
                // WORLD_REL = 7, WORLD_ABS = 11
                return getBaseFrame(handle);
            case 8:
            case 12:
                // WEAVE_REL = 8, WEAVE_ABS = 12
                return getWeaveFrame(handle);
            default:
                ROS_WARN("unknown reference frame for sns-control in mode %d", params.sensor_type);
                return "";
        }
    }

    std::string getReferenceFrame(GenericSnsTrkHandle &handle) {
        SensorTrackParams params;
        std::visit(overload{
                [&params](auto &h) { params = h.getSensorTrackParams(); }
        }, handle);
        return getReferenceFrame(params, handle);
    }

    bool isRelative(GenericSnsTrkHandle &handle) {
        SensorTrackParams params;
        std::visit(overload{
                [&params](auto &h) { params = h.getSensorTrackParams(); }
        }, handle);
        return (params.sensor_type >= 5) && (params.sensor_type < 9);
    }

    bool isAbsolute(GenericSnsTrkHandle &handle) {
        SensorTrackParams params;
        std::visit(overload{
                [&params](auto &h) { params = h.getSensorTrackParams(); }
        }, handle);
        return params.sensor_type >= 9;
    }


} // namespace