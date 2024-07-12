//
// Created by gabler on 26.01.21.
//

#ifndef HRR_CONTROLLERS_UTILS_H
#define HRR_CONTROLLERS_UTILS_H

#include <variant>
#include <string>
#include <sstream>

#include <eigen_conversions/eigen_msg.h>
#include <realtime_tools/realtime_publisher.h>
#include <comau_driver/comau_driver.h>
#include <comau_driver/comau_hw_interfaces/cartesian_state_interface.h>
#include <comau_driver/comau_hw_interfaces/sensor_tracking_command_interface.h>
#include <comau_driver/comau_hw_interfaces/joint_trajectory_command_interface.h>
#include <comau_driver/comau_hw_interfaces/cartesian_trajectory_command_interface.h>
#include <comau_driver/comau_hw_interfaces/digital_io_interface.h>
#include <comau_msgs/Digital.h>
#include <hrr_common/conversions.h>

namespace hrr_controllers {

  const Eigen::Vector3d ZeroVec{Eigen::Vector3d::Zero()};
  const Eigen::Quaterniond ZeroQuat{Eigen::Quaterniond::Identity()};

  using hardware_interface::comau::Translation;
  using hardware_interface::comau::Rotation;
  using hardware_interface::comau::PosCmd;
  using hardware_interface::comau::RotCmd;

  using comau_driver::SensorTrackParams;

  // short handles for readability
  // handles
  typedef hardware_interface::comau::CartesianStateHandle CartHandle;
  typedef hardware_interface::comau::SensorTrackingCommandHandle SnsTrkHandle;
  typedef hardware_interface::comau::SensorTrackingPoseCommandHandle SnsTrkPoseHandle;
  typedef hardware_interface::comau::TP5StateHandle TP5Handle;
  typedef hardware_interface::comau::TrajectoryCommandHandle JointTrajHandle;
  typedef hardware_interface::comau::CartesianTrajectoryCommandHandle CartTrajHandle;
  typedef hardware_interface::comau::CartesianPoseTrajectoryCommandHandle CartPoseTrajHandle;

  //interfaces
  typedef hardware_interface::comau::CartesianStateInterface CartIF;
  typedef hardware_interface::comau::SensorTrackingCommandInterface SnsTrkIF;
  typedef hardware_interface::comau::SensorTrackingPoseCommandInterface SnsTrkPoseIF;
  typedef hardware_interface::comau::TP5StateInterface TP5IF;
  typedef hardware_interface::comau::JointTrajectoryCommandInterface JointTrajF;
  typedef hardware_interface::comau::CartesianTrajectoryCommandInterface CartTrajF;
  typedef hardware_interface::comau::CartesianPoseTrajectoryCommandInterface CartPoseTrajIF;

  // variants for eased APIs
  typedef std::variant<SnsTrkHandle, SnsTrkPoseHandle> GenericSnsTrkHandle;
  typedef std::variant<GenericSnsTrkHandle, CartHandle> GenericCartHandle;
  typedef std::variant<JointTrajHandle, CartTrajHandle, CartPoseTrajHandle> GenericTrajHandle;
  typedef std::variant<JointTrajHandle, CartTrajHandle, CartPoseTrajHandle, TP5Handle> GenericTP5Handle;

  // RT Cart data
  typedef realtime_tools::RealtimePublisher<tf2_msgs::TFMessage> RtTfPub;
  typedef realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> RtWrenchPub;
  typedef std::shared_ptr<RtWrenchPub> RtWrenchPubPtr;

  template<class... Ts>
  struct overload : Ts ... {
    using Ts::operator() ...;
  };
  template<class... Ts> overload(Ts...) -> overload<Ts...>;

  inline Eigen::Vector3d scale_vector(const Eigen::Vector3d &vec, double vec_norm_limit) {
    double vec_norm{vec.norm()};
    if (vec_norm > vec_norm_limit) {
      ROS_DEBUG("clip vector (%.0f, %.2f, %.2f)e-3 by factor %.3f",
                vec[0] * 1e3, vec[1] * 1e3, vec[2] * 1e3,
                vec_norm_limit / vec_norm);
      return vec * vec_norm_limit / vec_norm;
    }
    return vec;
  }

  inline bool checkFrame(const std::string &msg_frame, const std::string &sns_frame) {
    if (msg_frame.empty())
      return true;
    if (msg_frame != sns_frame) {
      ROS_ERROR("reference frame %s different from Sensor Tracking reference frame %s",
                msg_frame.c_str(), sns_frame.c_str());
      return false;
    }
    return true;
  }

  inline void saveMsgToQuaternion(const geometry_msgs::Quaternion &m, Eigen::Quaterniond &e) {
    Eigen::Quaterniond q_tmp{};
    tf::quaternionMsgToEigen(m, q_tmp);
    if (q_tmp.norm() == 0.0) {
      ROS_ERROR("parsed empty quaternion [%.2f <<(%.2f, %.2f, %.2f)>>] -> assume identity",
                q_tmp.w(), q_tmp.x(), q_tmp.y(), q_tmp.z());
      e = Eigen::Quaterniond::Identity();
    } else {
      e = q_tmp;
    }
  }

  inline std::string pinToName(const int &pin) {
    if (pin > 0)
      return "DOUT" + std::to_string(pin);
    throw std::runtime_error("illegal pin number " + std::to_string(pin));
  }

  inline comau_msgs::Digital as_msg(const int &pin, const bool &value){
    comau_msgs::Digital tmp{};
    tmp.pin = pin;
    tmp.state = value;
    return tmp;
  }

  inline comau_msgs::Digital as_msg(const std::pair<int, bool> p_v){
    return as_msg(p_v.first, p_v.second);
  }

  inline comau_msgs::Digital as_msg(const hardware_interface::comau::DigitalPinCommandHandle &ph){
    comau_msgs::Digital tmp{};
    tmp.pin = ph.getPin();
    tmp.state = ph.getValue();
    return tmp;
  }

  inline comau_msgs::Digital as_msg(const std::shared_ptr<hardware_interface::comau::DigitalPinCommandHandle> &ph_ptr){
    comau_msgs::Digital tmp{};
    tmp.pin = ph_ptr->getPin();
    tmp.state = ph_ptr->getValue();
    return tmp;
  }

  inline void log_helper_stream(std::stringstream &ss, const std::vector<bool> &tmp){
    ss << "[";
    for (auto x: tmp){
      if (x){
        ss << "true, ";
      } else {
        ss << "false, ";
      }
    }
    ss.seekp(-2, std::ios_base::end);
    ss << "] ";
  }

  inline void  log_helper_stream(std::stringstream &ss, const std::vector<int> &tmp){
    ss << "[";
    for (auto &x: tmp){
      ss << x << ", ";
    }
    ss.seekp(-2, std::ios_base::end);
    ss << "] ";
  }


  inline void log_helper_stream(std::stringstream &ss, const std::vector<double> &tmp){
    ss << "[";
    for (auto &x: tmp){
      ss << x << ", ";
    }
    ss.seekp(-2, std::ios_base::end);
    ss << "] ";
  }



  inline std::string log_helper(const std::vector<double> &tmp){
    std::stringstream ss{};
    log_helper_stream(ss, tmp);
    return ss.str();
  }


}

#endif //HRR_CONTROLLERS_UTILS_H
