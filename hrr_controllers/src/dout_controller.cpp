#include <hrr_controllers/dout_controller.h>
#include <pluginlib/class_list_macros.h>


namespace hrr_controllers {

  struct DigitalPinsControllerParams {
    std::string get_state_srv_name{"get_state"};
    std::string set_state_srv_name{"set_state"};
    std::string reset_srv_name{"reset"};
    std::string invert_srv_name{"invert"};
    std::string frame_id;
    std::vector<int> pins{};
    std::vector<bool> values{};
    bool valid{false};

    explicit DigitalPinsControllerParams(const ros::NodeHandle &ctrl_nh) {
      ctrl_nh.param<std::string>("get_state_srv_name", get_state_srv_name, get_state_srv_name);
      ctrl_nh.param<std::string>("set_state_srv_name", set_state_srv_name, set_state_srv_name);
      ctrl_nh.param<std::string>("reset_srv_name", reset_srv_name, reset_srv_name);
      ctrl_nh.param<std::string>("invert_srv_name", invert_srv_name, invert_srv_name);
      ctrl_nh.param<std::string>("frame_id", frame_id, frame_id);
      std::vector<double> tmp_pins{};
      std::vector<double> tmp_values{};
      try {
        std::size_t error = 0;
        error += !rosparam_shortcuts::get(DigitalPinController::m_log_name, ctrl_nh, "pins", tmp_pins);
        error += !rosparam_shortcuts::get(DigitalPinController::m_log_name, ctrl_nh, "pin_start_values", tmp_values);
        if (error){
          std::runtime_error("could not read pins & start values from ROS-parameter server");
        } else{
          ROS_DEBUG_NAMED(DigitalPinController::m_log_name, "[%s] read pins %s with values %s", 
                          DigitalPinController::m_log_name.c_str(), log_helper(tmp_pins).c_str(),  log_helper(tmp_values).c_str());
        }
        ctrl_nh.param<std::vector<double >>("pins", tmp_pins);
        ctrl_nh.param<std::vector<double >>("pin_start_values", tmp_values);
        if (tmp_pins.empty()){
          throw std::runtime_error("no pins provided for ROS-parameter 'pins'");
        }
        if (tmp_values.empty()){
          throw std::runtime_error("no default values provided for ROS-parameter 'pin_start_values'");
        }
        if (tmp_pins.size() != tmp_values.size()) {
          throw std::runtime_error("number of pins must be equal to start state");
        }
        for (size_t i = 0; i < tmp_pins.size(); i++){
          pins.emplace_back((int) tmp_pins[i]);
          values.emplace_back(tmp_values[i] > 0.0);
        }
      } catch (ros::InvalidNameException &e) {
        ROS_ERROR_STREAM("["<< DigitalPinController::m_log_name <<"] " << e.what());
        return;
      } catch (std::runtime_error &e) {
        ROS_ERROR_STREAM("["<< DigitalPinController::m_log_name <<"] " << e.what());
        return;
      }
      valid = true;
    }

    [[nodiscard]] std::string print_params() const {
      std::stringstream ss{};
      ss << "\n\tget_state_srv_name:            \t" << get_state_srv_name
         << "\n\tset_state_srv_name:            \t" << set_state_srv_name
         << "\n\treset_srv_name:                \t" << reset_srv_name
         << "\n\tinvert_srv_name:               \t" << invert_srv_name
         << "\n\tframe_id:                      \t" << frame_id
         << "\n\tpins:                          \t[";
      for (auto &p: pins) {
        ss << std::to_string(p) << ", ";
      }
      ss << "]\n\tvalues:                      \t[";
      for (auto p: values) {
        ss << std::to_string(p) << ", ";
      }
      ss << "]";
      return ss.str();
    }
  };


  bool DigitalPinController::init(hardware_interface::comau::DigitalPinCommandInterface *hw, ros::NodeHandle &ctrl_nh) {
    DigitalPinsControllerParams params{ctrl_nh};
    if (params.valid) {
      ROS_DEBUG_NAMED(m_log_name, "[%s] ROS-parameters:%s", m_log_name.c_str(), params.print_params().c_str());
    } else {
      ROS_ERROR("[%s] invalid ROS-parameters:%s", m_log_name.c_str(), params.print_params().c_str());
      return false;
    }
    for (auto i = 0; i < (int) params.pins.size(); i++) {
      m_start_values[params.pins[i]] = params.values[i];
      m_pin_handles.insert(std::pair<int, DigPinHandle>{params.pins[i], hw->getHandle(pinToName(params.pins[i]))});
      ROS_DEBUG_NAMED(m_log_name, "[%s] added control handles for pin %d", m_log_name.c_str(), params.pins[i]);
    }
    m_ros_srvs.emplace_back(ros::ServiceServer(ctrl_nh.advertiseService(
        params.get_state_srv_name, &DigitalPinController::get_controller_state_srv, this))
    );
    m_ros_srvs.emplace_back(ros::ServiceServer(ctrl_nh.advertiseService(
        params.set_state_srv_name, &DigitalPinController::set_pins_srv, this))
    );

    m_ros_srvs.emplace_back(ros::ServiceServer(ctrl_nh.advertiseService(
        params.reset_srv_name, &DigitalPinController::reset_srv, this))
    );
    m_ros_srvs.emplace_back(ros::ServiceServer(ctrl_nh.advertiseService(
        params.invert_srv_name, &DigitalPinController::invert_srv, this))
    );
    m_frame_id = params.frame_id;
    return true;
  }

  void DigitalPinController::reset() {
    for (auto &[pin, ph]: m_pin_handles) {
      m_desired_values[pin] = m_start_values[pin];
      ph.setValue(m_start_values[pin]);
      m_error = false;
      m_enabled = false;
    }
  }

  void DigitalPinController::starting(const ros::Time &time) {
    reset();
  }

  void DigitalPinController::stopping(const ros::Time &time1) {
    reset();
  }


  void DigitalPinController::update(const ros::Time &time, const ros::Duration &period) {
    for (auto &[pin, ph]: m_pin_handles) {
      m_current_values[pin] = ph.getValue();
      if (m_current_values[pin] != m_desired_values[pin])
        ph.setValue(m_desired_values[pin]);
    }
  }


  bool DigitalPinController::get_controller_state_srv(hrr_msgs::GetDoutControllerStateRequest &req,
                                                      hrr_msgs::GetDoutControllerStateResponse &res) {
    res.header.frame_id = m_frame_id;
    res.header.stamp = ros::Time::now();
    res.enabled = m_enabled;
    res.error = m_error;
    res.pins.clear();
    for (auto &p_v: m_current_values) {
      res.pins.emplace_back(as_msg(p_v));
    }
    if (std::equal(m_desired_values.cbegin(), m_desired_values.cend(), m_start_values.cbegin())){
      res.msg = "default / idle";
    } else{
      res.msg = "active / modified";
    }
    return true;
  }

  bool DigitalPinController::set_pins_srv(hrr_msgs::SetPinsRequest &req, hrr_msgs::SetPinsResponse &res) {
    if (m_error) {
      res.success = false;
      res.msg = "cannot set pins in error mode -> reset first.";
      return true;
    }
    m_enabled = (req.state == req.ENABLED);
    m_error = (req.state == req.ERROR);
    for (auto &digital: req.pins_desired) {
      m_desired_values[digital.pin] = digital.state;
    }
    res.success = true;
    return true;
  }

  bool DigitalPinController::invert_srv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    if (m_error) {
      res.success = false;
      res.message = "controller in error-state -> reset first";
      return false;
    }
    for (auto &[pin, value]: m_desired_values) {
      m_desired_values[pin] = !value;
    }
    res.success = true;
    res.message = "reset current DOUT pins";
    return true;
  }

  bool DigitalPinController::reset_srv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
    if (m_error) {
      m_error = false;
      res.message = "successfully reset controller";
    } else {
      res.message = "controller is not in error-state";
    }
    res.success = true;
    reset();
    return true;
  }

}
PLUGINLIB_EXPORT_CLASS(hrr_controllers::DigitalPinController, controller_interface::ControllerBase);