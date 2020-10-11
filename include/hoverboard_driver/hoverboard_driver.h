#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "uart_peripheral.h"

#include "hoverboard_driver/serial.h"
#include "hoverboard_driver/hoverboard_msg.h"

class hoverboard_driver : public uart_peripheral
{
public:
    explicit hoverboard_driver(char* device);
    virtual ~hoverboard_driver();
private:
    void velCallback(const geometry_msgs::Twist &vel);
};