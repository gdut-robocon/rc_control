//
// Created by jialonglong on 23-7-13.
//

#pragma once
#include <serial/serial.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include "action_manager.h"

namespace rc_hw
{
    class DT35_laser
    {
    public:
        bool init(std::string serialPort);
        void readDT35_laser(const ros::Time &time, const ros::Duration &period);
    protected:
    private:
        void unPack(const uint8_t data);
        union receiveData
        {
         int d;
         unsigned char data[4];
        }laserData;
        std::vector<rc_hw::SerialFlamMark> serial_flam_marks_;
        std::vector<int> serial_fds_{};
        int fd;
    };
}//namespace rc_hw
