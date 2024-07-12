cmake_minimum_required(VERSION 3.0.2)
project(hrr_controllers)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

# check c++17 / c++11 / c++0x
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
if (COMPILER_SUPPORTS_CXX17)
    message(STATUS "Compile with C++17 support")
    add_compile_options(-std=c++17)
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 (+) support. Please update the pkg build-essential ")
endif ()
set(CMAKE_CXX_STANDARD 17)

find_package(
        catkin REQUIRED COMPONENTS
        comau_msgs
        comau_driver
        controller_interface
        control_toolbox
        dynamic_reconfigure
        geometry_msgs
        hardware_interface
        hrr_common
        hrr_msgs
        pluginlib
        realtime_tools
        roscpp
        tf
        )

catkin_python_setup()

generate_dynamic_reconfigure_options(
        cfg/Gains.cfg
)
#find_package(Eigen3 REQUIRED COMPONENTS system)
catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        hrr_controllers
        CATKIN_DEPENDS
        comau_driver
        comau_msgs
        control_toolbox
        controller_interface
        dynamic_reconfigure
        geometry_msgs
        hardware_interface
        hrr_common
        hrr_msgs
        pluginlib
        realtime_tools
        roscpp
        tf
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(hrr_controllers
        src/core/sns_trk_api.cpp
        src/core/traj_api.cpp
        src/cartesian_state_controller.cpp
        src/digital_io_state_controller.cpp
        src/dout_controller.cpp
        src/dout_encoder_controller.cpp
        src/tp5_state_controller.cpp
        src/ft_state_controller.cpp
        # src/kolver_controller.cpp
        src/sns_trk_velocity_controller.cpp
        src/sns_trk_hybrid_vel_force_controller.cpp
        # src/vacuum_gripper_controller.cpp
        )

add_dependencies(hrr_controllers
          ${${PROJECT_NAME}_EXPORTED_TARGETS}
          ${catkin_EXPORTED_TARGETS}
          ${PROJECT_NAME}_gencfg
        )
target_link_libraries(hrr_controllers
        ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES}
        )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN ".svn"
        EXCLUDE
        )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
