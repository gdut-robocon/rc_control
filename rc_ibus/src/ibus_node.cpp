//
// Created by myx on 2022/11/29.
//
#include "rc_ibus/ibus_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rc_ibus");
  IBusNode ibus_node;
  ros::Rate loop_rate(1000);
  while (ros::ok())
  {
    ibus_node.run();
    loop_rate.sleep();
  }
  return 0;
}

IBusNode::IBusNode()
{
  ibus_pub_ = nh_.advertise<rc_msgs::IbusData>("ibus_data", 1);
  nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
  ibus_.init(serial_port_.data());
}

void IBusNode::run()
{
  ibus_.read();
  ibus_.getData(&Ibus_cmd_);
  ibus_pub_.publish(Ibus_cmd_);
}
