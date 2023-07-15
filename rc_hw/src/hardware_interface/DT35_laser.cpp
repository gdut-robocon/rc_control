//
// Created by jialonglong on 23-7-13.
//

#include <rc_hw/hardware_interface/DT35_laser.h>

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
        //TO set some basic data of serial
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
        uint8_t read_byte;
        int timeout = 0;  // time out of one package
        while (timeout < 5)
        {
            // Read a byte //
            size_t n = ::read(fd, &read_byte, sizeof(read_byte));
            if (n == 0)
            {
                timeout++;
            }
            else if (n == 1)
            {
                unPack(read_byte);
            }
        }
    }

    void DT35_laser::unPack(const uint8_t data)
    {
        static unsigned char USARTBufferIndex     = 0;
        static short j=0,k=0;
        static short dataLength                   = 0;
        unsigned char buffer[22]={0};
        //check header
        if (data == 0x11)
        {
            buffer[22]=0;
            USARTBufferIndex = 0;
        }
        switch(USARTBufferIndex)
        {
            //check the length
            case 0:
                buffer[2]=0xaa;
                dataLength = buffer[2];
                USARTBufferIndex++;
                break;
            //check all data
            case 1:
                buffer[j+3]=0xaa;
                j++;
                if (j>=dataLength-1)
                {
                    j=0;
                    USARTBufferIndex++;
                }
                break;
            case 2:
                buffer[2+dataLength]=0xaa;
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
                        DT35_laser::laserData.data[k] = buffer[k+3];
                        DT35_laser::laserData.data[k] = buffer[k+7];
                        DT35_laser::laserData.data[k] = buffer[k+11];
                        DT35_laser::laserData.data[k] = buffer[k+15];
                    }
                    USARTBufferIndex = 0 ;
                    dataLength = 0 ;
                    j = 0 ;
                    k = 0 ;
                }
                break;
            default:
                break;
        }
    }
}//namespace rc_hw