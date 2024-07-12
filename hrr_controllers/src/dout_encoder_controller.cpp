#include <hrr_controllers/dout_encoder_controller.h>
#include <pluginlib/class_list_macros.h>


namespace hrr_controllers {

  struct DoutEncoderParams {
    std::string get_state_srv_name{"get_state"};
    std::string set_encoder_state_srv_name{"set_encoder_state"};
    std::string frame_id;
    std::vector<int> encoder_pin_nums{};
    std::vector<int> legal_out{};
    std::map<std::string, int> programs{};
    int encoder_size{3};
    int enable_encoder_pin{-1};
    bool valid{false};

    explicit DoutEncoderParams(const ros::NodeHandle &ctrl_nh) {
      ctrl_nh.param<std::string>("get_state_srv_name", get_state_srv_name, get_state_srv_name);
      ctrl_nh.param<std::string>("set_state_srv_name", set_encoder_state_srv_name, set_encoder_state_srv_name);
      ctrl_nh.param<int>("encoder_size", encoder_size, encoder_size);
      auto limit{(int) std::pow(2, encoder_size)};
      ctrl_nh.param<std::string>("frame_id", frame_id, frame_id);
      std::vector<std::string> program_names{};
      ctrl_nh.param<std::vector<std::string>>("program_names", program_names, program_names);
      try {
        std::vector<double> program_numbers;
        std::vector<double> tmp_pins;
        std::vector<double> tmp_legal_out{};
        std::size_t error = 0;
        error += !rosparam_shortcuts::get(DoutEncoderController::m_log_name, ctrl_nh, "encoder_pins", tmp_pins);
        rosparam_shortcuts::get(DoutEncoderController::m_log_name, ctrl_nh, "legal_programs", tmp_legal_out);
        rosparam_shortcuts::get(DoutEncoderController::m_log_name, ctrl_nh, "program_numbers", program_numbers);
        if (error) {
          throw std::runtime_error("could not read encoder pins from ROS-parameter server");
        } else {
          ROS_DEBUG_NAMED(DoutEncoderController::m_log_name, "[%s] read encoder pins %s and legal programs %s",
                          DoutEncoderController::m_log_name.c_str(), log_helper(tmp_pins).c_str(),
                          log_helper(tmp_legal_out).c_str());
        }
        if ((int) tmp_pins.size() != encoder_size) {
          throw std::runtime_error("encoder is of fixed length " + std::to_string(encoder_size) +
                                   ". Received " + std::to_string(tmp_pins.size()));
        }
        for (auto i = 0; i < encoder_size; i++) {
          encoder_pin_nums.emplace_back((int) tmp_pins[i]);
        }
        if (tmp_legal_out.size() > (size_t) limit) {
          throw std::runtime_error(
              "output is limited to 8. Received " + std::to_string(program_names.size()) + " legal out values.");
        }

        if (program_names.size() != program_numbers.size()) {
          throw std::runtime_error("number of program names " + std::to_string(program_names.size()) +
                                   " is inconsistent with number of programs " +
                                   std::to_string(program_numbers.size()));
        }
        if (program_names.size() > (size_t) std::pow(2, encoder_size)) {
          throw std::runtime_error("output is limited to 8. Received " + std::to_string(program_names.size()));
        }
        if (program_names.empty()) { tmp_legal_out.clear(); }
        for (auto i = 0UL; i < program_names.size(); i++) {
          tmp_legal_out.emplace_back((int) program_numbers[i]);
          programs[program_names[i]] = (int) program_numbers[i];
        }
        if (tmp_legal_out.empty()) {
          legal_out.clear();
          for (int i=1; i <= limit; i++) {
            legal_out.emplace_back(i);
          }
        } else {
          for (double i : tmp_legal_out) {
            if ((i < 0) || (i > limit)) {
              throw std::runtime_error("encoder enable pin is illegal: " + std::to_string(enable_encoder_pin));
            }
            legal_out.emplace_back((int) i);
          }
        }
        ctrl_nh.getParam("enable_encoder_pin", enable_encoder_pin);
        if (enable_encoder_pin < 0) {
          throw std::runtime_error("encoder enable pin is illegal: " + std::to_string(enable_encoder_pin));
        }
      } catch (ros::InvalidNameException &e) {
        ROS_ERROR_STREAM("[" << DoutEncoderController::m_log_name << "] " << e.what());
        return;
      } catch (std::runtime_error &e) {
        ROS_ERROR_STREAM("[" << DoutEncoderController::m_log_name << "] " << e.what());
        return;
      }
      valid = true;
    }

    [[nodiscard]] std::string print_params() const {
      std::stringstream ss{};
      ss << "\n\tget_state_srv_name:            \t" << get_state_srv_name
         << "\n\tset_encoder_state_srv_name:    \t" << set_encoder_state_srv_name
         << "\n\tframe_id:                      \t" << frame_id
         << "\n\tencoder_size:                  \t" << encoder_size
         << "\n\tenable_encoder_pin:            \t" << enable_encoder_pin
         << "\n\tlegal_out                      \t";
      log_helper_stream(ss, legal_out);
      ss << "\n\tencoder_pin_nums               \t";
      log_helper_stream(ss, encoder_pin_nums);
      ss << "\n\tprograms:                \t{";
      for (auto &[name, out]: programs) {
        ss << name << ": " << out << ", ";
      }
      ss << "}";
      return ss.str();
    }
  };

  bool
  DoutEncoderController::init(hardware_interface::comau::DigitalPinCommandInterface *hw, ros::NodeHandle &ctrl_nh) {
    DoutEncoderParams params{ctrl_nh};
    if (params.valid) {
      ROS_DEBUG_NAMED(m_log_name, "[%s] ROS-parameters:%s", m_log_name.c_str(), params.print_params().c_str());
    } else {
      ROS_ERROR("[%s] invalid ROS-parameters:%s", m_log_name.c_str(), params.print_params().c_str());
      return false;
    }
    m_encoder_pins.clear();
    m_encoder_size = params.encoder_size;
    for (auto i = 0; i < m_encoder_size; i++) {
      m_encoder_pins.emplace_back(hw->getHandle(pinToName(params.encoder_pin_nums[i])));
      ROS_DEBUG_NAMED(m_log_name, "[%s] registered %s as program-pin %d", m_log_name.c_str(),
                      pinToName(params.encoder_pin_nums[i]).c_str(), i + 1);
    }
    m_enable_encoder_pin = std::make_shared<DigPinHandle>(hw->getHandle(pinToName(params.enable_encoder_pin)));
    ROS_DEBUG_NAMED(m_log_name, "[%s] registered %s as program encoder enable pin", m_log_name.c_str(),
                    pinToName(params.enable_encoder_pin).c_str());
    m_ros_srvs.emplace_back(ros::ServiceServer(ctrl_nh.advertiseService(
        params.get_state_srv_name, &DoutEncoderController::get_controller_state_srv, this))
    );
    m_ros_srvs.emplace_back(ros::ServiceServer(ctrl_nh.advertiseService(
        params.set_encoder_state_srv_name, &DoutEncoderController::set_encoder_state_srv, this))
    );
    m_frame_id = std::move(params.frame_id);
    m_state_map = std::move(params.programs);
    m_legal_out = std::move(params.legal_out);
    return true;
  }


  bool DoutEncoderController::get_controller_state_srv(hrr_msgs::GetDoutControllerStateRequest &req,
                                                       hrr_msgs::GetDoutControllerStateResponse &res) {
    res.header.frame_id = m_frame_id;
    res.header.stamp = ros::Time::now();
    res.enabled = m_desired_out > 0;
    res.error = m_desired_out < 0;
    res.pins.clear();
    for (auto &pinHandle: m_encoder_pins) {
      res.pins.emplace_back(as_msg(pinHandle));
    }
    res.pins.emplace_back(as_msg(m_enable_encoder_pin));
    if (res.enabled) {
      for (auto &[name, pr]: m_state_map) {
        if (pr == m_current_out) {
          res.msg = "current program " + name + "(" + std::to_string(m_current_out) + ")";
          break;
        }
      }
      if (res.msg.empty())
        res.msg = "current program " + std::to_string(m_current_out);
    } else {
      res.msg = "idle";
    }
    return true;
  }

  bool DoutEncoderController::set_encoder_state_srv(hrr_msgs::SetEncoderStateRequest &req,
                                                    hrr_msgs::SetEncoderStateResponse &res) {
    int tmp_out{-1};
    try {
      tmp_out = m_state_map.at(req.state_name) - 1;
      res.msg = "set program to " + req.state_name + ", i.e. " + std::to_string(tmp_out);
      res.success = true;
    } catch (std::out_of_range &e) {
      if ((req.new_state < 0)) {
        res.msg = "Disable controller";
        res.success = true;
        tmp_out = -1;
      } else if (req.new_state > std::pow(2, m_encoder_size)) {
        res.msg =
            "program " + std::to_string(req.new_state) + " is out of limits, neither is " + req.state_name +
            " known. Disable encoder";
        res.success = false;
        tmp_out = -1;
      } else {
        tmp_out = (int) (uint8_t) req.new_state - 1;
        res.msg = "successfully updated encoder state to: " + std::to_string(m_desired_out);
        res.success = true;
      }
    }
    if ((tmp_out >= 0) && (std::find(begin(m_legal_out), end(m_legal_out), tmp_out + 1) == m_legal_out.end())) {
      res.msg = "output " + std::to_string(tmp_out + 1) + "is not allowed. Disable encoder";
      res.success = false;
      tmp_out = -1;
    }
    m_desired_out = tmp_out;
    ROS_DEBUG_NAMED(m_log_name, "[%s] set new program to %d from '%s' and %d", m_log_name.c_str(),
                    m_desired_out, req.state_name.c_str(), req.new_state);
    if (res.success) {
      m_set_time = ros::Time::now();
      m_timeout = (req.timeout > 0) ? ros::Duration{req.timeout} : ros::Duration{1e6};
    }
    return true;
  }

  void DoutEncoderController::update(const ros::Time &time, const ros::Duration &period) {
    m_current_out = 0;
    for (auto i = 0; i < m_encoder_size; i++) {
      if (m_encoder_pins[i].getValue()) {
        m_current_out += (int) std::pow(2, i);
      }
    }
    if (m_enable_encoder_pin->getValue()) {
      m_current_out = -1;
    }
    if ((m_desired_out >= 0) && (m_timeout > ros::Duration(0)) && (time - m_set_time > m_timeout)) {
      m_desired_out = -1;
    }
    m_enable_encoder_pin->setValue(m_desired_out < 0);
    if (m_current_out != m_desired_out) {
      auto tmp{m_desired_out};
      for (auto i = m_encoder_size - 1; i >= 0; i--) {
        auto cur = (int) std::pow(2, i);
        if (tmp >= cur) {
          m_encoder_pins[i].setValue(true);
          tmp -= cur;
        } else {
          m_encoder_pins[i].setValue(false);
        }
      }
    }
  }

  void DoutEncoderController::reset() {
    m_current_out = -1;
    m_desired_out = -1;
    m_set_time = ros::Time::now();
    m_enable_encoder_pin->setValue(true);
  }

  void DoutEncoderController::starting(const ros::Time &time) {
    reset();
  }

  void DoutEncoderController::stopping(const ros::Time &time1) {
    reset();
  }

}
PLUGINLIB_EXPORT_CLASS(hrr_controllers::DoutEncoderController, controller_interface::ControllerBase)