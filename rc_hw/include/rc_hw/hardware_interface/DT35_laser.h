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
#include <rc_common/hardware_interface/dt35_interface.h>
#include <rc_common/filters/filters.h>
namespace rc_hw
{
    class DT35_laser
    {
    public:
        bool init(std::string serialPort);
        void readDT35_laser(const ros::Time &time, const ros::Duration &period);

        std::vector<rc_control::SharpIR> dt35_data_values;
    protected:
    private:
        void unPack(const uint8_t data);
        struct dt35_laser
        {
            int data;
        }dt35Laser1,dt35Laser2,dt35Laser3,dt35Laser4;
        union receiveData
        {
         int d;
         unsigned char data[4];
        }laserData1,laserData2,laserData3,laserData4;
        std::vector<rc_hw::SerialFlamMark> serial_flam_marks_;
        std::vector<int> serial_fds_{};
        int fd;
        RampFilter<double>*laser1_{}, *laser2_{}, *laser3_{},*laser4_{};
    };
}//namespace rc_hw
