#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include "hoverboard_driver/serial.h"

using namespace std;

namespace hoverboard_driver_node {
    class Hoverboard {

    };
}

int main(int argc, char **argv) {

    // Start ROS node.
    ROS_INFO("Starting node");
    ros::init(argc, argv, "hoverboard_driver");
    ros::NodeHandle node;
    ros::Rate rate(10);  // 10 hz
}