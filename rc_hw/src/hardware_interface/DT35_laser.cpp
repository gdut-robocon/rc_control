//
// Created by jialonglong on 23-7-13.
//

#include <rc_hw/hardware_interface/DT35_laser.h>
#include <rc_common/hardware_interface/dt35_interface.h>

namespace rc_hw
{
    bool DT35_laser::init(std::string serialPort)
    {
        char* port = (char*)serialPort.data();
        fd = open(port,O_RDWR | O_NOCTTY | O_SYNC);
        if(fd < 0)
        {
            ROS_ERROR_STREAM("Unable to open serial of DT35. Serial port:" << serialPort);
            return false;
        }
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0)
        {
            ROS_ERROR_STREAM("Unable to get serial port attributes");
            return false;
        }
        //To set some basic data of serial
        cfsetospeed(&tty,B115200);//bitrate
        cfsetispeed(&tty,B115200);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        if (tcsetattr(fd,TCSANOW,&tty) !=0)
        {
            ROS_ERROR_STREAM("Unable to get serial port attributes");
            return false;
        }
        serial_fds_.push_back(fd);
        ROS_INFO_STREAM("Successful to open " << serialPort << " port.");
        return true;
    }

    void DT35_laser::readDT35_laser(const ros::Time &time, const ros::Duration &period)
    {
        uint8_t buffer[1];
        int timeout = 0;  // time out of one package
        while (timeout < 5)
        {
            // Read a byte with a timeout of 100 ms //
            fd_set set;
            struct timeval timeout_tv;
            FD_ZERO(&set);
            FD_SET(fd, &set);
            timeout_tv.tv_sec = 0;
            timeout_tv.tv_usec = 100000;  // 100 ms
            int select_ret = select(fd + 1, &set, NULL, NULL, &timeout_tv);
            if (select_ret == -1)
            {
                // Error in select()
                ROS_ERROR_STREAM("Error in select() function");
                return;
            }
            else if (select_ret == 0)
            {
                // Timeout
                timeout++;
            }
            else
            {
                // Data is available to read
                size_t n = ::read(fd, buffer, 1);
                if (n == 1)
                {
                    unPack(buffer[0]);
                }
            }
        }
    }


    void DT35_laser::unPack(const uint8_t data)
    {
        static unsigned char USARTBufferIndex= 0;
        static short j=0,k=0;
        static short dataLength=0;
        static unsigned char buffer[22]={0};
        const int buffer_size=22;
        unsigned char header[2]={0x55,0xaa};
        const unsigned char ender[2]={0x0d,0x0a};
        static bool start= true;
        //check header
        if (start)
        {
            if (data == 0x55)
            {
                buffer[0]=header[0];
            }
            if (data == 0xaa)
            {
                buffer[1]=header[1];
                USARTBufferIndex=0;
                start = false;
            }
            else
            {
                buffer[0]=buffer[1];
            }
        }
        else
        {
            switch(USARTBufferIndex)
            {
                //check the length
                case 0:
                    buffer[2]=data;
                    dataLength = buffer[2];
                    USARTBufferIndex++;
                    break;
                    //check all data
                case 1:
                    buffer[j+3]=data;
                    j++;
                    if (j>=dataLength-1)
                    {
                        j=0;
                        USARTBufferIndex++;
                    }
                    break;
                case 2:
                    buffer[2+dataLength]=data;
                    USARTBufferIndex++;
                    break;
                case 3:
                    if(k==0)
                    {
                        k++;
                    }
                    else if (k==1)
                    {
                        for (k=0;k<4;k++)
                        {
                            if (k + 3 < buffer_size)
                                DT35_laser::laserData1.data[k] = buffer[k + 3]; // laser_1
                            else
                                ROS_WARN("laser_1 stack smashing detected,please check your settings");
                            if (k + 7 < buffer_size)
                                DT35_laser::laserData2.data[k] = buffer[k + 7]; // laser_2
                            else
                                ROS_WARN("laser_2 stack smashing detected,please check your settings");
                            if (k + 11 < buffer_size)
                                DT35_laser::laserData3.data[k] = buffer[k + 11]; // laser_3
                            else
                                ROS_WARN("laser_3 stack smashing detected,please check your settings");
                            if (k + 15 < buffer_size)
                                DT35_laser::laserData4.data[k] = buffer[k + 15]; // laser_4
                            else
                                ROS_WARN("laser_4 stack smashing detected,please check your settings");
                        }
                        laser1_->input(DT35_laser::laserData1.d);
                        laser2_->input(DT35_laser::laserData2.d);
                        laser3_->input(DT35_laser::laserData3.d);
                        laser4_->input(DT35_laser::laserData4.d);
                        DT35_laser::dt35Laser1.data=laser1_->output();
                        DT35_laser::dt35Laser2.data=laser2_->output();
                        DT35_laser::dt35Laser3.data=laser3_->output();
                        DT35_laser::dt35Laser4.data=laser4_->output();
                        USARTBufferIndex = 0 ;
                        dataLength = 0 ;
                        j = 0 ;
                        k = 0 ;
                        start =true ;
                    }
                    break;
                default:
                    break;
            }
        }
        }

}//namespace rc_hw