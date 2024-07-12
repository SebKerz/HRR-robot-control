cmake_minimum_required(VERSION 3.0.2)
project(hrr_common)

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

find_package(catkin REQUIRED COMPONENTS        
        geometry_msgs
        hardware_interface
        roscpp
		rosparam_shortcuts
        tf2_eigen
        tf2_ros        
        )


find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_python_setup()
 
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS  
	  geometry_msgs 
	  hardware_interface   
	  roscpp  
	  rosparam_shortcuts
	  tf2_eigen 
	  tf2_ros        
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/utils.cpp
        src/conversions.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)


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

install (PROGRAMS
   scripts/*
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

