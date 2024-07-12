#include <hrr_cobot_robot/hrr_cobot_hw.h>
#include <hrr_cobot_robot/hrr_safety.h>
#include <controller_manager/controller_manager.h>


namespace hrr_cobot{

    typedef std::tuple<std::shared_ptr<hrr_cobot::HrrCobotHW>, std::shared_ptr<controller_manager::ControllerManager>> NodeCoreContent;

    bool connectToRobot(ros::NodeHandle &nh, ros::NodeHandle &nh_local,
                        std::shared_ptr<hrr_cobot::HrrCobotHW> &hrr_cobot_hw,
                        const int &num_tries=10, const double &sleep_time=2.0){
        ROS_DEBUG_STREAM("[" << ros::this_node::getName() << "] trying to connect to robot-HW interface ");
        for (auto i=0;i < num_tries; i++){
            if (hrr_cobot_hw->init(nh, nh_local)) {
                ROS_INFO("[%s] robot-HW interface initialized", ros::this_node::getName().c_str());
                return true;
            }
            ROS_WARN("[%s] failed to initialize robot. sleep for %f seconds", ros::this_node::getName().c_str(), sleep_time);
            ros::Duration{sleep_time}.sleep();
        }
        return false;
    }


    bool initControllerManager(std::shared_ptr<hrr_cobot::HrrCobotHW> &hrr_cobot_hw,
                               ros::NodeHandle &nh_local,
                               std::shared_ptr<controller_manager::ControllerManager> &controller_manager){
        try{
            controller_manager.reset(new controller_manager::ControllerManager(hrr_cobot_hw.get(), nh_local));
            ROS_DEBUG("[%s] controller manager initialized", ros::this_node::getName().c_str());
            return true;
        } catch (const std::runtime_error& e) {
            ROS_ERROR_STREAM("[" << ros::this_node::getName() << "] error while initializing controller-manager" << e.what());
            return false;
        } catch (...){
            ROS_DEBUG("[%s] unknown error while initializing controller-manager", ros::this_node::getName().c_str());
            return false;
        }
    }

    void printInterfaces(const std::shared_ptr<hrr_cobot::HrrCobotHW> &hrr_cobot_hw){
        ROS_INFO_STREAM("[" << ros::this_node::getName() << "] HW interface initialized");
        for (auto& comau_if_name : std::vector<std::string>{
                "SensorTrackingCommandInterface", "SensorTrackingPoseCommandInterface", "JointTrajectoryCommandInterface",
                "CartesianTrajectoryCommandInterface", "CartesianPoseTrajectoryCommandInterface", "DigitalIOStateInterface",
                "DigitalOUTCommandInterface", "DigitalPinCommandInterface", "DigitalPinStateInterface",
                "CartesianStateInterface", "TP5StateInterface" })
        {
            for (auto& iface : hrr_cobot_hw->getInterfaceResources("hardware_interface::comau::" + comau_if_name))
            {
            ROS_DEBUG("[%s] Registered: %s->%s", ros::this_node::getName().c_str(), comau_if_name.c_str(), iface.c_str());
            }
        }
    }

    ros::Rate getRate(const ros::NodeHandle &nh_local, const std::string &name){
        std::size_t error = 0;
        double loop_hz{500.0};
        error += !rosparam_shortcuts::get(name, nh_local, "loop_hz", loop_hz);
        rosparam_shortcuts::shutdownIfError(name, error);
        return  ros::Rate {loop_hz};
    }

     bool initRobot(ros::NodeHandle &nh, ros::NodeHandle &nh_local,
                 std::shared_ptr<hrr_cobot::HrrCobotHW> &hrr_cobot_hw){
        // connect to robot
        if (!connectToRobot(nh, nh_local, hrr_cobot_hw)){
            ROS_ERROR("[%s] failed to connect to robot hardware",
                      ros::this_node::getName().c_str());
            return false;
        }
        printInterfaces(hrr_cobot_hw);
        return true;
    }

    bool initCore(ros::NodeHandle &nh, ros::NodeHandle &nh_local,
                 std::shared_ptr<hrr_cobot::HrrCobotHW> &hrr_cobot_hw,
                 std::shared_ptr<controller_manager::ControllerManager> &controller_manager,
                 const double& t_wait=1.0){
        // connect to robot
        if (!initRobot(nh, nh_local, hrr_cobot_hw)){
            return false;
        }
        if (t_wait > 0.0)
            ros::Duration{t_wait}.sleep();
        // start controller manager
        if (!initControllerManager(hrr_cobot_hw, nh_local, controller_manager))
            ROS_ERROR("[%s] failed to start controller-manager",
                      ros::this_node::getName().c_str());
            return false;
        return true;
    }

    void sleep(const bool& valid, ros::Rate& default_rate,
              const double &retry_rate=1.0){
        if (valid)
            default_rate.sleep();
        else{
            ROS_ERROR("[%s] encountered error in control loop. Retry in %.2f second",
                      ros::this_node::getName().c_str(), 1.0 / retry_rate);
            ros::Rate{retry_rate}.sleep();
        }
    }

}