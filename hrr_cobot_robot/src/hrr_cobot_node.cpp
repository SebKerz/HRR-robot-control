#include <hrr_cobot_robot/hrr_cobot_node_utils.h>


using namespace hrr_cobot;




#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <comau_driver/comau_hw_control_loop.h>
#include <comau_driver/comau_robot_hw.h>
#include <csignal>



int main(int argc, char **argv) {
    const std::string log_name{"hrr_cobot_node"};
    std::string name{"hrr_cobot_node"};
    ros::init(argc, argv, name);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    auto rate = getRate(nh_local, name);

    std::shared_ptr<hrr_cobot::HrrCobotHW> hrr_cobot_hw = std::make_shared<hrr_cobot::HrrCobotHW>();

    // Create the hardware interface
    if (!(initRobot(nh, nh_local,hrr_cobot_hw))){
        ROS_ERROR("[%s] failed to initialize robot handle", ros::this_node::getName().c_str());
        exit(1);
    }
    ros::Duration{1.0}.sleep();
    ROS_DEBUG_NAMED(log_name,"[%s] initializing controller manager", ros::this_node::getName().c_str());
    std::shared_ptr<controller_manager::ControllerManager> controller_manager {nullptr};
    if (initControllerManager(hrr_cobot_hw, nh_local, controller_manager)){
        ROS_INFO_NAMED(log_name,"[%s] Initialized controller manager.", ros::this_node::getName().c_str());
    } else{
        ROS_ERROR_NAMED(log_name,"[%s] Failed to initialize controller manager. Exiting", ros::this_node::getName().c_str());
        ros::Duration(1.0);
        exit(1);
    }
    rate.sleep();
    auto prev = ros::Time::now();
    while (ros::ok()) {
        auto now = ros::Time::now();
        auto elapsed_time = now - prev;
        auto valid = false;
        // Default Control cycle step
        try{
            hrr_cobot_hw->read(now, elapsed_time);
            controller_manager->update(now, elapsed_time);
            hrr_cobot_hw->write(now, elapsed_time);
            valid = true;
        } catch (const std::runtime_error& e) {
            ROS_ERROR_STREAM("error during cobot loop: " << e.what());
        } catch (...){}
        sleep(valid, rate);
        prev = now;
    }
    ROS_ERROR_STREAM("shutting down...");
    hrr_cobot_hw.reset();
    controller_manager.reset();
    return 0;

}