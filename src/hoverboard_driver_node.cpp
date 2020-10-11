#include "hoverboard_driver/hoverboard_driver.h"

using namespace std;

namespace hoverboard_driver_node {

    class Hoverboard {
    public:

        typedef struct{
            uint16_t	start;
            int16_t  steer;
            int16_t  speed;
            uint16_t checksum;
        } SerialCommand;
        SerialCommand command;

        typedef struct{
            uint8_t start;
            int16_t 	cmd1;
            int16_t 	cmd2;
            int16_t 	speedR_meas;
            int16_t 	speedL_meas;
            int16_t 	errorR;
            int16_t 	errorL;
            int16_t 	batVoltage;
            int16_t 	boardTemp;
            uint16_t cmdLed;
            uint16_t checksum;
        } SerialFeedback;

        SerialFeedback feedback;

        Hoverboard(std::string serial_name) : serial_name_(serial_name) {
            serial_ = serial_new();

            if (serial_open(serial_, serial_name.c_str(), 38400) < 0) {
                ROS_ERROR("serial_open(): %s\n", serial_errmsg(serial_));
                exit(1);
            }
        }

        hoverboard_driver::hoverboard_msg read_data(bool* error) {
            if (serial_read(serial_, hoverboard_data, 1, 20) < 0) {
                *error = true;
            };

            if (hoverboard_data[0] == 0xCD) {

                if (serial_read(serial_, (uint8_t *)&feedback, 21, 100) < 0) {
                    *error = true;
                }

            }
            hoverboard_driver::hoverboard_msg msg;
            msg.cmd1 = feedback.cmd1;
            msg.cmd2 = feedback.cmd2;
            msg.speedR_meas = feedback.speedR_meas;
            msg.speedL_meas = feedback.speedL_meas;
            msg.errorR = feedback.errorR;
            msg.errorL = feedback.errorL;
            msg.batVoltage = feedback.batVoltage;
            msg.boardTemp = feedback.boardTemp;
            msg.cmdLed = feedback.cmdLed;

            *error = false;

            return msg;

        };

        void sendCommand();

        void close() {
            serial_close(serial_);
            serial_free(serial_);
        };

    private:
        std::string serial_name_;
        serial_t *serial_;
        uint8_t hoverboard_data[22];
    };

} // namespace hoverboard_driver_node

hoverboard_driver_node::Hoverboard *hoverboard_instance;

void velCallback(const geometry_msgs::Twist &vel) {
    //hoverboard_instance.sendCommand()
}

void publish_odometry(){

}

void publishMessage(ros::Publisher odrive_pub, hoverboard_driver::hoverboard_msg msg){
    // Publish message
    odrive_pub.publish(msg);
}

int main(int argc, char **argv) {
    // Start ROS node.
    ROS_INFO("Starting hoverboard_driver node");
    ros::init(argc, argv, "hoverboard_driver");
    ros::NodeHandle node;
    ros::Rate rate(10);  // 10 hz

    std::string hoverboard_uart;

    node.param<std::string>("hoverboard_uart", hoverboard_uart, "/dev/ttyTHS1");

    hoverboard_driver_node::Hoverboard hoverboard(hoverboard_uart);

    hoverboard_instance = &hoverboard;

    ros::Publisher hoverboard_pub = node.advertise<hoverboard_driver::hoverboard_msg>("hoverboard_msg", 100);

    ros::Publisher hoverboard_odometry = node.advertise<nav_msgs::Odometry>("odometry", 100);

    ros::Subscriber hoverboard_cmd_vel = node.subscribe("cmd_vel", 10, velCallback);

    tf::TransformBroadcaster odom_broadcaster;

    bool hoverboard_error = false;

    while (ros::ok()) {
        hoverboard_driver::hoverboard_msg feedback = hoverboard.read_data(&hoverboard_error);

        if(hoverboard_error){
            ROS_ERROR("Can't connect to hoverboard!");
        } else {
            publishMessage(hoverboard_pub, feedback);
            //publish_odometry(feedback);
        }

        ros::spinOnce();
        rate.sleep();
    }

    hoverboard.close();
}