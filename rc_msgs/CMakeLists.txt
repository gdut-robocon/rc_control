cmake_minimum_required(VERSION 3.0.2)
project(rc_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        actionlib
        actionlib_msgs
        )

add_message_files(
        FILES
        ActuatorState.msg
        GpioData.msg
        LpData.msg
        ActionData.msg
        ChassisCmd.msg
)

add_service_files(
        FILES
        ActionCmd.srv
        #        ContinousDetectorSwitch.srv
        #        StatusChange.srv
        #        CameraStatus.srv
        #        EnableImuTrigger.srv
)

#add_action_files(
#        FILES
#        Engineer.action
#)

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        actionlib_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        actionlib
        actionlib_msgs
)

