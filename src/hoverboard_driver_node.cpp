#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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

        void read_data() {

        };

        void close() {
            serial_close(serial_);
            serial_free(serial_);
        };

    private:
        std::string serial_name_;
        serial_t *serial_;
    };

} // namespace hoverboard_driver_node

void velCallback(const geometry_msgs::Twist &vel) {

}

int main(int argc, char **argv) {
    // Start ROS node.
    ROS_INFO("Starting node");
    ros::init(argc, argv, "hoverboard_driver");
    ros::NodeHandle node;
    ros::Rate rate(10);  // 10 hz

    std::string hoverboard_uart;

    node.param<std::string>("hoverboard_uart", hoverboard_uart, "/dev/ttyTHS1");

    hoverboard_driver_node::Hoverboard hoverboard(hoverboard_uart);

    ros::Publisher hoverboard_odometry = node.advertise<nav_msgs::Odometry>("odometry", 100);

    ros::Subscriber hoverboard_cmd_vel = node.subscribe("cmd_vel", 10, velCallback);

    tf::TransformBroadcaster odom_broadcaster;

    while (ros::ok()) {
        hoverboard.read_data();
        ros::spinOnce();
        rate.sleep();
    }

    hoverboard.close();
}