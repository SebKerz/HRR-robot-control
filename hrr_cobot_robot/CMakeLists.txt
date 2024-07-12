cmake_minimum_required(VERSION 3.0.2)
project(hrr_cobot_robot)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

# check c++17
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
if (COMPILER_SUPPORTS_CXX17)
    message(STATUS "Compile with C++17 support")
    add_compile_options(-std=c++17)
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++ support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif ()


find_package(catkin
    REQUIRED COMPONENTS
        comau_driver
        controller_manager
        dynamic_reconfigure
        force_torque_plugin
        hardware_interface
        hrr_common
        hrr_msgs
        pluginlib
        roscpp
        rosparam_shortcuts
        std_msgs
)


catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/SkillGraph.cfg
)


catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
    comau_driver
    force_torque_plugin
    hardware_interface
    hrr_common
    hrr_msgs
    controller_manager
    pluginlib
    roscpp
    rosparam_shortcuts
    std_srvs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# Comau Driver
add_library(hrr_cobot_hw
        src/hrr_cobot_hw.cpp
        src/hrr_safety.cpp
    )
add_dependencies(hrr_cobot_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hrr_cobot_hw ${catkin_LIBRARIES})


# Comau Hardware Interface node
add_executable(hrr_cobot_node src/hrr_cobot_node.cpp)
add_dependencies(hrr_cobot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hrr_cobot_node hrr_cobot_hw ${catkin_LIBRARIES})

# # Comau Hardware Interface node
add_executable(hrr_cobot_secure_node src/hrr_cobot_secure_node.cpp)
add_dependencies(hrr_cobot_secure_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hrr_cobot_secure_node hrr_cobot_hw ${catkin_LIBRARIES})


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )


catkin_install_python(PROGRAMS scripts/devel/dummyDevTypePub.py scripts/devel/dummyDevCenterPub.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
