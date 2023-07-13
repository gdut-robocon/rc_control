//
// Created by myx on 2022/11/29.
//

#pragma once

#include "ibus.h"
#include <ros/ros.h>
#include <rc_msgs/IbusData.h>
namespace rc_ibus {
    class IBusNode {
    private:
        ros::NodeHandle nh_;
        ros::Publisher ibus_pub_;
        std::string serial_port_;
        rc_msgs::IbusData Ibus_cmd_;
        rc_ibus::IBus ibus_{};

    public:
        IBusNode();

        ~IBusNode() = default;

        void run();
    };
}//namespace rc_ibus