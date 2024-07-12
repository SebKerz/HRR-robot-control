#include <hrr_cobot_robot/hrr_cobot_node_utils.h>


using namespace hrr_cobot;


int main(int argc, char **argv) {
    std::string name{"hrr_cobot_node"};
    ros::init(argc, argv, name);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    std::shared_ptr<controller_manager::ControllerManager> controller_manager{};
    std::shared_ptr<hrr_cobot::HrrCobotHW> hrr_cobot_hw = std::make_shared<hrr_cobot::HrrCobotHW>();

    auto rate = getRate(nh_local, name);
    assert(initCore(nh, nh_local,hrr_cobot_hw, controller_manager));

    // init safety watchdog layer
    SafetyWatchDog watchdog {nh, ""}; 
    auto prev = ros::Time::now();
    while (ros::ok()) {
       if (!watchdog.update(hrr_cobot_hw, controller_manager)){
            ROS_ERROR("Safety guard raised error in update. Force exit...");
            return 1;
        }
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
    return 0;
}