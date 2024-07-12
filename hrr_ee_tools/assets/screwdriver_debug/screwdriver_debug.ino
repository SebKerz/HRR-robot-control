 /*
 * @file general_ee_tools.ino
 * @author Annalena Daniels (a.daniels@tum.de)
 * @author Klara Tomaskovic (klara.tomaskovic@gmail.com)
 * @author Volker gabler (v.gabler@tum.de)
 * @brief HR-Recycler Tool control script
 * @version 0.1
 * @date 2021-12-15
 * This short arduino code enables an Arduino to control

 *  * PIN  6: a Shaft grinder via PWM motor control using a 10V control line
 *  * PIN 13: a vacuum gripper controlled via a pneumatic vacuum switch
 *  * PIN 0-5 & 7-12: a screwdriver KOLVER pluto 10
 * @copyright Copyright (c) 2021 TUM-LSR HR-Recycler Group
 */
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

using std_srvs::SetBool;
using std_srvs::Empty;


#define SC_STOP 0  // STOP SCREWDRIVER PIN I/O
#define SC_START 1 // SCREWDRIVER START PIN I/O
#define SC_REVERSE 2 // SCREWDRIVER REVERSE PIN I/O
#define SC_PRG1 12 // SCREWDRIVER PROGRAM 1 PIN I/O
#define SC_PRG2 11 // SCREWDRIVER PROGRAM 2 PIN I/O
#define SC_PRG3 10 // SCREWDRIVER PROGRAM 3 PIN I/O
#define SC_PRG4 9 // SCREWDRIVER PROGRAM 4 PIN I/O
#define SC_PRG5 8 // SCREWDRIVER PROGRAM 5 PIN I/O
#define SC_PRG6 7 // SCREWDRIVER PROGRAM 6 PIN I/O
#define SC_PRG7 5 // SCREWDRIVER PROGRAM 7 PIN I/O
#define SC_PRG8 4 // SCREWDRIVER PROGRAM 8 PIN I/O

#define GRINDER_SWITCH 6 // GRINDER PIN (needs PWM)
#define VACUUM_SWITCH 13 // VACUUM PIN

#define GR 1
#define SC 2
#define VC 3
#define OFF -2
#define NONE 0

// globals
bool grinder_on = false;
bool emergency = false;
bool vacuum_on = false;
bool screwdriver_on = false;
int16_t t_last_msg, t;
int16_t t_last_screwdriver_msg, t_last_vacuum_msg, t_last_grinder_msg;
int16_t T_switch = 50;
std_msgs::Int8 driver_state;
// Grinder Control
uint8_t PWM = 0;
// screwdriver control
int8_t sc_prog = -1;
bool sc_start= false;
bool sc_reverse = false;
bool sc_stop = false;

// debug content
std_msgs::Float32 debug_data;


/**
 * @brief grinder control -> convert RPM to PWM
 * 
 * @param rpm set value in rotations/minute
 * @param min_pwm_i8 minimum pwm number. Values below are ignored, i.e. 0.
 * @param max_pwm_i8 maximum pwm number. Values above are cut to 255.
 * @return uint8_t pwm value to set grinder to speed of rpm
 */
uint8_t rpm_to_pwm(const int16_t& rpm,
                   const uint8_t& min_pwm_i8 = 20,
                   const uint8_t& max_pwm_i8 = 255) {
    float max_rpm=25000.0;
    float min_rpm=3500.0;
    float max_pwm = (float) max_pwm_i8;
    float min_pwm = (float) min_pwm_i8;
    uint8_t pwm = (uint8_t) (
      (max_pwm - min_pwm) / (max_rpm-min_rpm) * (float) rpm + // rpm_to_pwm_slope
      (max_rpm*min_pwm-min_rpm*max_pwm)/(max_rpm-min_rpm)     // rpm_to_pwm_coefficient
      );
    if (pwm > max_pwm_i8){
      return max_pwm_i8;
    } else if (pwm < min_pwm_i8){
      return 0;
    } else{
      return pwm;
    }
}

/**
 * @brief Set the grinder speed object
 * 
 * @param grinder_pwm_speed ROS-msg as rmp speed value
 */
void set_grinder_speed(const std_msgs::Int8& grinder_rpm_speed){
  if (!emergency){
    PWM = rpm_to_pwm(grinder_rpm_speed.data + 128); // int8 -> unsigned int8
    t_last_msg = t;
    t_last_grinder_msg = t;
  } else{
    grinder_on= false;
  }
}


/**
 * @brief Set the screwdriver button cb object
 * values:
 *  - 1 -> enable program with set motion direction
 *  - 2 -> enable program with reversed motion
 *  - default -> trigger stop
 * @param button ROS-message as button
 */
void set_screwdriver_button_cb(const std_msgs::Int8& button) {
  if (screwdriver_on) {
    switch (button.data) {
      case SC_START:
        sc_start = true;
        sc_reverse = false;
        sc_stop = false;
        break;sc_start
      case SC_REVERSE:
        sc_start = false;
        sc_reverse = true;
        sc_stop = false;
        break;
      case SC_STOP:
        sc_start = false;
        sc_reverse = false;
        sc_stop = true;
      default:
        sc_start = false;
        sc_reverse = false;
        sc_stop = false;
        break;
    }
  }
  t_last_msg = t;
  t_last_screwdriver_msg = t;
}

/**
 * @brief Set the screwdriver prog cb object
 * 
 * ignore values outside of (1,8)
 * @param button ROS-message for desired program
 */
void set_screwdriver_prog_cb(const std_msgs::Int8& button) {
  if (screwdriver_on) {
    if ((button.data > 0) && (button.data <= 8)){
      sc_prog = button.data;
    }
  }
  t_last_msg = t;
  t_last_screwdriver_msg = t;
}

/**
 * @brief Screwdriver -> helper function to set I/O pin values
 * switch for program
 * and set start / reverse / stop afterwards
 * @note: this function does not check if the arduino is in `screwdriver` mode
 */
void write_screwdriver_pins(){
  if (sc_prog == 1){
    digitalWrite(SC_PRG1, HIGH);
  } else{
    digitalWrite(SC_PRG1, LOW);
  }
  if (sc_prog == 2){
    digitalWrite(SC_PRG2, HIGH);
  } else{
    digitalWrite(SC_PRG2, LOW);
  }
  if (sc_prog == 3){
    digitalWrite(SC_PRG3, HIGH);
  } else{
    digitalWrite(SC_PRG3, LOW);
  }
  if (sc_prog == 4){
    digitalWrite(SC_PRG4, HIGH);
  } else{
    digitalWrite(SC_PRG4, LOW);
  }
  if (sc_prog == 5){
    digitalWrite(SC_PRG5, HIGH);
  } else{
    digitalWrite(SC_PRG5, LOW);
  }
  if (sc_prog == 6){
    digitalWrite(SC_PRG6, HIGH);
  } else{
    digitalWrite(SC_PRG6, LOW);
  }
  if (sc_prog == 7){
    digitalWrite(SC_PRG7, HIGH);
  } else{
    digitalWrite(SC_PRG7, LOW);
  }
  if (sc_prog == 8){
    digitalWrite(SC_PRG8, HIGH);
  } else{
    digitalWrite(SC_PRG8, LOW);
  }
  if (sc_start){
    digitalWrite(SC_START, HIGH);
    digitalWrite(SC_REVERSE, LOW);
  }
  else if (sc_reverse){
    digitalWrite(SC_START, LOW);
    digitalWrite(SC_REVERSE, HIGH);
  }
  if (sc_stop){
    digitalWrite(SC_STOP, HIGH);
  } else { 
    digitalWrite(SC_STOP, LOW);
  }
}


/**
 * @brief shared function -> reset set values
 */
void reset_values(){
  emergency = false;
  t = 0;
  t_last_msg = -1;
  t_last_screwdriver_msg = -T_switch;
  t_last_vacuum_msg = -T_switch;
  t_last_grinder_msg = -T_switch;
}

/**
 * @brief shared function -> reset emergency state via ROS-service
 */
void reset_srv_cb(const Empty::Request & req, Empty::Response & res){
  if (emergency){
    reset_values();
  }
}


/** 
 * @brief helper function
 * disable screwdriver mode if last screwdriver message is older than 0.5 seconds
 */
bool disable_sc(){
  if (t - t_last_screwdriver_msg > T_switch){
      screwdriver_on = false;
      return true;
  }
  return false;
}

/** @brief helper function
 *  disable vacuum gripper mode if last vacuum gripper message is older than 0.5 seconds
 */
bool disable_vc(){
  if (t - t_last_vacuum_msg > T_switch){
      vacuum_on = false;
      return true;
  }
  return false;
}

/** @brief helper function
 *  disable shaft grinder mode if last vacuum gripper message is older than 0.5 seconds
 */
bool disable_gr(){
  if (t - t_last_grinder_msg > T_switch){
      grinder_on = false;
      return true;
  }
  return false;
}


/** @brief shared function -> set control mode
 *  Set control mode via an Int8 ROS-msg
 */
void set_tool_mode_cb(const std_msgs::Int8& mode){
  if (emergency){
    return;
  }
  switch (mode.data) {
      case GR:
        if (screwdriver_on){
          if (disable_sc()){
            grinder_on = true;
          }
        } else if (vacuum_on){
          if (disable_vc()){
            grinder_on = true;
          }
        } else {
          grinder_on = true;
        }
        t_last_grinder_msg = t;
        break;
      case SC:
        if (grinder_on){
          if (disable_gr()){
            screwdriver_on = true;
          }
        } else if (vacuum_on){
          if (disable_vc()){
            screwdriver_on = true;
          }
        } else {
          screwdriver_on = true;
        }
        t_last_screwdriver_msg = t;
        break;
      case VC:
        if (grinder_on){
          if (disable_gr()){
            vacuum_on = true;
          }
        } else if (screwdriver_on){
          if (disable_sc()){
            vacuum_on = true;
          }
        } else {
          vacuum_on = true;
        }
        t_last_vacuum_msg = t;
        break;
      default:
        emergency = true;
        grinder_on = false;
        screwdriver_on = false;
        vacuum_on = false;
        break;
    }
    t_last_msg = t;
}


ros::NodeHandle  nh;
// shared ROS-API
ros::ServiceServer<Empty::Request, Empty::Response> reset_srv("~reset", &reset_srv_cb);
ros::Subscriber<std_msgs::Int8> sub_tool_mode("~set_control_mode", &set_tool_mode_cb);
ros::Publisher pub_driver_status("~status", &driver_state);

// grinder control
ros::Subscriber<std_msgs::Int8> grinder_speed_sub("~grinder_speed", &set_grinder_speed);

// screwdriver control
ros::Subscriber<std_msgs::Int8> sub_screwdriver_button_cb("~scredriver/motor", &set_screwdriver_button_cb);
ros::Subscriber<std_msgs::Int8> sub_screwdriver_mode_cb("~scredriver/set_program", &set_screwdriver_prog_cb);

// debug out
ros::Publisher pub_debug_data("~debug_sc_data", &debug_data);

/**
 * @brief Set the pins of the Arduino
 */
void set_pins()
{

  // set LEDs
  //   pinMode(LED, OUTPUT);
  //   pinMode(RED_LED, OUTPUT);
  //   pinMode(YELLOW_LED, OUTPUT);
  //   digitalWrite(RED_LED, HIGH);
  //   digitalWrite(LED, HIGH);
  // set grinder
  pinMode(GRINDER_SWITCH, OUTPUT);
  analogWrite(GRINDER_SWITCH, 0);

  // set vacuum
  pinMode(VACUUM_SWITCH, OUTPUT);
  digitalWrite(VACUUM_SWITCH, LOW);

  // set screwdriver
  pinMode(SC_START, OUTPUT);
  pinMode(SC_REVERSE, OUTPUT);
  pinMode(SC_PRG1, OUTPUT);
  pinMode(SC_STOP, OUTPUT);
  pinMode(SC_PRG1, OUTPUT);
  pinMode(SC_PRG2, OUTPUT);
  pinMode(SC_PRG3, OUTPUT);
  pinMode(SC_PRG4, OUTPUT);
  pinMode(SC_PRG5, OUTPUT);
  pinMode(SC_PRG6, OUTPUT);
  pinMode(SC_PRG7, OUTPUT);
  pinMode(SC_PRG8, OUTPUT);
  
  write_screwdriver_pins();
}

/**
 * @brief setup overall program
 * 
 * - set pins
 * - set ROS API
 * - reset values
 */
void setup()
{
  set_pins();
  nh.initNode();
  // shared functions
  nh.advertiseService(reset_srv);
  nh.advertise(pub_driver_status);
  nh.subscribe(sub_tool_mode);

  //grinder
  nh.subscribe(grinder_speed_sub);

  //screwdriver
  nh.subscribe(sub_screwdriver_button_cb);
  nh.subscribe(sub_screwdriver_mode_cb);

  nh.advertise(pub_debug_data);

  reset_values();
}



/** 
 * @brief debugging function
 * publish internal states to ROS as needed
 */
void ros_debug(){
  if (screwdriver_on){
    debug_data.data = sc_prog;
    if (sc_start){
      debug_data.data += 100;
    }
    else if (sc_reverse){
      debug_data.data *= -1;
    } else if (sc_stop){
      debug_data.data *= 0.5;
    }
    pub_debug_data.publish(&debug_data);
  }
}

/**
 * @brief main loop iteration
 * run one cycle of current control Arduino code
 * 
 * - write data if any set value has been obtained and driver is not in emergency state
 * - check against current state
 * - reset current state in case of timeout
 * - publish driver state
 * - publish debug data
 * - end cycle by sleep
 */
void loop()
{
  int16_t T_delay = 10;
  int16_t T_MAX = 2000;
  nh.spinOnce();
  driver_state.data = NONE;
  if (t_last_msg >= 0) {
    if (emergency) {
      digitalWrite(VACUUM_SWITCH, LOW);
      digitalWrite(SC_STOP, HIGH);
      analogWrite(GRINDER_SWITCH, 0);
      driver_state.data = OFF;
    } else {
      // write grinder data to arduino
      if (grinder_on) {
        digitalWrite(SC_STOP, HIGH);
        digitalWrite(VACUUM_SWITCH, LOW);
        if (disable_gr()){
          analogWrite(GRINDER_SWITCH, 0);
        } else {
          analogWrite(GRINDER_SWITCH, PWM);
          driver_state.data = GR;
        }
      // write vacuum data to arduino
      } else if (vacuum_on) {
        digitalWrite(SC_STOP, HIGH);
        analogWrite(GRINDER_SWITCH, 0);
        if (disable_vc()){
          digitalWrite(VACUUM_SWITCH, LOW);
        } else {
          digitalWrite(VACUUM_SWITCH, HIGH);
          driver_state.data = VC;
        }
      } else if (screwdriver_on) {
        analogWrite(GRINDER_SWITCH, 0);
        digitalWrite(VACUUM_SWITCH, LOW);
        if (disable_sc()){
          digitalWrite(SC_STOP, HIGH);
        }
        else{
          write_screwdriver_pins();
          driver_state.data = SC;
        }
      }
    }
    t += 1;
  }
  pub_driver_status.publish(&driver_state);
  ros_debug();
  delay(T_delay);
}
