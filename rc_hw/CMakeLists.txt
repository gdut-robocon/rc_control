cmake_minimum_required(VERSION 3.0.2)
project(rc_hw)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint

        rc_common
        rc_msgs
        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager
        urdf
        realtime_tools
        serial
        )

## Find system libraries
#find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}_loader_plugins
        CATKIN_DEPENDS
        roscpp
        roslint

        rc_common
        rc_msgs
        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager
        urdf
        realtime_tools
        serial
        DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
        src/hardware_interface/hardware_interface.cpp
        src/hardware_interface/parse.cpp
        src/hardware_interface/socketcan.cpp
        src/hardware_interface/can_bus.cpp
        src/hardware_interface/gpio_manager.cpp
        src/hardware_interface/action_manager.cpp
        src/control_loop.cpp
        src/hardware_interface/DT35_laser.cpp)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
#install(
#        FILES rm_hw_loader_plugins.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rc_hw.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

roslint_cpp()
