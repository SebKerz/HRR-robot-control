//
// Created by gabler on 27.01.21.
//
#include <hrr_common/conversions.h>

namespace hrr_cobot {

    Eigen::Quaterniond zeroOrderDifference(const Eigen::Vector3d &omega, double dt) {
        double norm{omega.norm()};
        if (norm > 1e-12) {
            double w{cos(norm * dt * 0.5)};
            Eigen::Vector3d quat_vec {omega / norm * sin(norm * dt * 0.5)};
            return Eigen::Quaterniond(w, quat_vec.x(),quat_vec.y(),quat_vec.z());
        }
        return Eigen::Quaterniond{1., 0., 0., 0.};
    }

    Eigen::Quaterniond zeroOrderIntegrateAngular(const Eigen::Vector3d &omega,
                                                 double dt,
                                                 const Eigen::Quaterniond &q_cur){
        return q_cur * zeroOrderDifference(omega, dt);
    }

    Eigen::Vector3d zeroOrderIntegrate(const Eigen::Vector3d& x_dot, double dt, const Eigen::Vector3d x_cur){
        return x_cur + x_dot * dt;
    }


    Eigen::Quaterniond quatError(const Eigen::Quaterniond &q_des,
                                 const Eigen::Quaterniond &q_cur,
                                 float max_rot){
        auto delta = q_cur.angularDistance(q_des);
        auto t = std::max(0.0, std::min(max_rot / delta, 1.0));
        return q_cur.slerp(t, q_des).conjugate() * q_cur;
    }

    Eigen::Matrix<double, 3, 3> rotMatError(const Eigen::Quaterniond &q_des,
                                            const Eigen::Quaterniond &q_cur,
                                            float max_rot){
        return quatError(q_des, q_cur, max_rot).toRotationMatrix();
    }


    Eigen::Vector3d rotError(const Eigen::Quaterniond &q_des,
                             const Eigen::Quaterniond &q_cur,
                             float max_rot){
        auto dq = quatError(q_des, q_cur, max_rot);
        if (dq.w() >1e-10) {
            return Eigen::Vector3d{dq.x() / dq.w(), dq.y() / dq.w(), dq.z() / dq.w()};
        } else{
            return Eigen::Vector3d{dq.x(), dq.y(), dq.z()};
        }
    }


    std::array<double, 6> poseToArray(const geometry_msgs::Point &d_pos, const geometry_msgs::Quaternion &d_quat){
        double roll, pitch, yaw;
        KDL::Rotation tmp{KDL::Rotation::Quaternion(d_quat.x, d_quat.y, d_quat.z, d_quat.w)};
        tmp.GetRPY(roll, pitch, yaw);        
        return std::array<double, 6>{
            d_pos.x, d_pos.y, d_pos.z,
            yaw, pitch, roll
        };
    }

    std::tuple<Eigen::Vector3d, Eigen::Quaterniond> get_tf_helper(const std::shared_ptr<tf2_ros::Buffer> &tf2_bfr,
                                                                         const std::string &frame_A, const std::string &frame_B){
        Eigen::Vector3d A_p_AB{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond quat_A_B{Eigen::Quaterniond::Identity()};
        try{
            auto T_A_B = tf2_bfr->lookupTransform(frame_A, frame_B, ros::Time(0));
            tf::vectorMsgToEigen(T_A_B.transform.translation, A_p_AB);
            tf::quaternionMsgToEigen(T_A_B.transform.rotation, quat_A_B);
        } catch (const tf2::LookupException& e){
            ROS_ERROR_STREAM_THROTTLE(1.0, "frame " << frame_A << "not known: " << e.what());
        }
        return std::make_tuple(A_p_AB, quat_A_B);
    }
}
