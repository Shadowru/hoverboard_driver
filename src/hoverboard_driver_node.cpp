#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include "hoverboard_driver/serial.h"

using namespace std;

namespace hoverboard_driver_node {
    class Hoverboard {
    public:
        Hoverboard(std::string serial_name) : serial_name_(serial_name) {
                serial_ = serial_new();

                if (serial_open(serial_, serial_name.c_str(), 38400) < 0) {
                    ROS_ERROR("serial_open(): %s\n", serial_errmsg(serial_));
                    exit(1);
                }
        }

    };
} //namespace hoverboard_driver_node

int main(int argc, char **argv) {
    // Start ROS node.
    ROS_INFO("Starting node");
    ros::init(argc, argv, "hoverboard_driver");
    ros::NodeHandle node;
    ros::Rate rate(10);  // 10 hz

    std::string hoverboard_uart;

    node.param<std::string>("hoverboard_uart", hoverboard_uart, "/dev/ttyTHS1");

    hoverboard_driver_node::Hoverboard hoverboard(hoverboard_uart);

    ros::Publisher hoverboard_odometry = nh.advertise<nav_msgs::Odometry>("odometry", 100);

    ros::Subscriber hoverboard_cmd_vel = nh.subscribe("cmd_vel", 10, velCallback);

    tf::TransformBroadcaster odom_broadcaster;

}