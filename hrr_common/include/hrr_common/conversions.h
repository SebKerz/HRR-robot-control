//
// Created by gabler on 27.01.21.
//

#ifndef HRR_COMMON_CONVERSIONS_H
#define HRR_COMMON_CONVERSIONS_H

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <kdl/frames.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_listener.h>



namespace hrr_cobot {
    /**
     * Calculates the relative quaternion for a zero-order hold integration step
     *
     *      q {ωn∆t} := [ ω / ||ω|| sin( 0.5 * ||ω|| * ∆t ),
     *                              cos( 0.5 * ||ω|| * ∆t ) ]
     *
     * Refer to https://arxiv.org/pdf/1711.02508.pdf page 17+ for technical insights
     * @param omega ω as angular velocity in rad/s in R^3
     * @param dt ∆t as update step size in s.
     * @return q {ωn∆t} from aboge
     */
    Eigen::Quaterniond zeroOrderDifference(const Eigen::Vector3d& omega, double dt);

    /**
     * Calculates the relative quaternion for a zero-order hold integration step
     *
     *      q_{n+1} =q_{n} ⊗ q {ωn∆t}
     *
     * Refer to https://arxiv.org/pdf/1711.02508.pdf page 17+ for technical insights
     * @param omega ω as angular velocity in rad/s in R^3
     * @param dt ∆t as update step size in s.
     * @return q {ωn∆t} from aboge
     */
    Eigen::Quaterniond zeroOrderIntegrateAngular(const Eigen::Vector3d& omega, double dt,
                                                 const Eigen::Quaterniond &q_cur = Eigen::Quaterniond{1., 0., 0., 0.});

    /**
     * @brief helper function to adjust integration methods as needed
     * @param x_{t} current value
     * @param xdot as the temproal derivative of x
     * @param dt ∆t as update step size in s.
     * @return x_{t+1}
     */
    Eigen::Vector3d zeroOrderIntegrate(const Eigen::Vector3d& x_dot, double dt,
                                       const Eigen::Vector3d& x);



    /**
     * Calculate Quaternion-error from quaternion q1 -> q2 using a maximal displacement threshold
     *
     * @note if threshold is too large, the conversion may suffer from gimbal locks
     * @param q_cur current quaternion
     * @param q_des desired quaternion
     * @param delta_rot_max maximum displacement in rad
     * @return scaled quaternion error from q_cur to q_cur
     */
    Eigen::Quaterniond quatError(const Eigen::Quaterniond &q_des,
                                 const Eigen::Quaterniond &q_cur,
                                 double max_rot);
    /**
     * Calculate Rotation Matrix-error from quaternion q1 -> q2 using a maximal displacement threshold
     *
     * @note if threshold is too large, the conversion may suffer from gimbal locks
     * @param q_cur current quaternion
     * @param q_des desired quaternion
     * @param delta_rot_max maximum displacement in rad
     * @return scaled rotation matrix from q_cur to q_des
     */
    Eigen::Matrix<double, 3, 3> rotMatError(const Eigen::Quaterniond &q_des,
                                            const Eigen::Quaterniond &q_cur,
                                            double max_rot);


    /**
     * Calculate RPY-error from quaternion q1 -> q2 using a maximal displacement threshold
     *
     * @note if threshold is too large, the conversion may suffer from gimbal locks
     * @param q_cur current quaternion
     * @param q_des desired quaternion
     * @param delta_rot_max maximum displacement in rad
     * @return roll-pitch-yaw error around (x,y,z) in rad
     */
    Eigen::Vector3d rotError(const Eigen::Quaterniond &q_des,
                             const Eigen::Quaterniond &q_cur,
                             float max_rot);


    /**
     * @brief convert Eigen pose to pose-array that can be parsed to sensor-track command interface
     * 
     * @param d_pos delta position offset
     * @param d_quat delta quaternion offset
     * @return pose with rpy configuration as handled by comau-robots
     */
    std::array<double, 6> poseToArray(const geometry_msgs::Point &d_pos, const geometry_msgs::Quaternion &d_quat);

    /**
     * Helper function to get latest transofrmation of two frames
     * Returns transformation from B -> A, i.e.
     *
     *      {}^{A}\mathbf{T}_{B}
     * as
     *
     *      {}^{A}\mathbf{p}_{AB}
     *
     *  and quaternion
     *
     *      \mathbf{q}^{A}_{B}
     *
     * @param tf2_bfr tf2 ros buffer
     * @param frame_A name of frame A for tf2-ROS buffer
     * @param frame_B name of frame B for tf2-ROS buffer
     * @return
     */
    std::tuple<Eigen::Vector3d, Eigen::Quaterniond> get_tf_helper(const std::shared_ptr<tf2_ros::Buffer> &tf2_bfr,
                                                                  const std::string &frame_A, const std::string &frame_B);




    }
#endif //HRR_COMMON_CONVERSIONS_H