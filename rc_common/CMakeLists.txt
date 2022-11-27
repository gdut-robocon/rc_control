cmake_minimum_required(VERSION 3.0.2)
project(rc_common)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-enum-compare)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint
        tf
        rc_msgs
        geometry_msgs
        control_msgs
        controller_manager_msgs
        imu_complementary_filter
        imu_filter_madgwick
        realtime_tools
        dynamic_reconfigure
        )

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIR}
        CATKIN_DEPENDS
        tf
        rc_msgs
        geometry_msgs
        control_msgs
        controller_manager_msgs
        imu_complementary_filter
        imu_filter_madgwick
        roscpp
        dynamic_reconfigure
        DEPENDS
        LIBRARIES
        rc_common
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

file(GLOB_RECURSE sources "src/*.cpp" "src/filter/*.cpp")

add_library(rc_common SHARED ${sources})
#add_executable(test_traj test/test_traj.cpp)
#add_executable(test_kalman test/test_kalman_filter.cpp)

target_link_libraries(rc_common ${catkin_LIBRARIES})
#target_link_libraries(test_traj rc_common ${catkin_LIBRARIES})
#target_link_libraries(test_kalman rc_common ${catkin_LIBRARIES})

# Fix rc_msgs generation problem
# See https://answers.ros.org/question/73048
add_dependencies(rc_common rc_msgs_generate_messages_cpp)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
#install(
#        DIRECTORY doc
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rc_common.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
roslint_cpp()
