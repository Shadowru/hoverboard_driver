#include "hoverboard_driver/hoverboard_driver.h"

using namespace std;

namespace hoverboard_driver_node {

    class Hoverboard {
    public:

        typedef struct {
            uint16_t start;
            int16_t steer;
            int16_t speed;
            uint16_t checksum;
        } SerialCommand;
        SerialCommand command;

        typedef struct {
            uint8_t start;
            int16_t cmd1;
            int16_t cmd2;
            int16_t speedR_meas;
            int16_t speedL_meas;
            int16_t errorR;
            int16_t errorL;
            int16_t batVoltage;
            int16_t boardTemp;
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

        hoverboard_driver::hoverboard_msg read_data(bool *error) {
            if (serial_read(serial_, hoverboard_data, 1, 20) < 0) {
                *error = true;
            };

            if (hoverboard_data[0] == 0xCD) {
                if (serial_read(serial_, hoverboard_data, 21, 100) < 0) {
                    *error = true;
                }
            }

            hoverboard_driver::hoverboard_msg msg;

            int idx = 1;

            msg.cmd1 = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);//feedback.cmd1;
            msg.cmd2 = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);//feedback.cmd2;
            msg.speedR_meas = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);//feedback.speedR_meas;
            msg.speedL_meas = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);//feedback.speedL_meas;
            msg.errorR = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);//feedback.errorR;
            msg.errorL = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);//feedback.errorL;
            msg.batVoltage = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);//feedback.batVoltage;
            msg.boardTemp = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);//feedback.boardTemp;
            msg.cmdLed = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);//feedback.cmdLed;

            uint16_t msg_checksum = hoverboard_data[19] + (hoverboard_data[20]<< 8);

            uint16_t calc_checksum = 0xABCD ^ msg.cmd1 ^ msg.cmd2 ^ msg.speedR_meas ^ msg.speedL_meas ^ msg.batVoltage ^ msg.boardTemp ^ msg.cmdLed;

            if(msg_checksum != calc_checksum){
                ROS_ERROR("Checksum wrong!: ");
                *error = true;
            } else {
                *error = false;
            }

            return msg;
        };

        bool sendCommand(uint16_t steer, uint16_t speed){
            uint8_t hoverboard_command[8];
            uint16_t start = 0xABCD;
            uint16_t checksum = start ^ steer ^ speed;
            int idx = 0;
            hoverboard_command[idx++]= start & 0xff;
            hoverboard_command[idx++]=(start >> 8);
            hoverboard_command[idx++]= steer & 0xff;
            hoverboard_command[idx++]=(steer >> 8);
            hoverboard_command[idx++]= speed & 0xff;
            hoverboard_command[idx++]=(speed >> 8);
            hoverboard_command[idx++]= checksum & 0xff;
            hoverboard_command[idx++]=(checksum >> 8);

            if(serial_write(serial_, hoverboard_command, 8) < 0){
                ROS_ERROR("Write command error!");
                return false;
            };

            return true;
        };

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

void setInstance(hoverboard_driver_node::Hoverboard *instance){
    hoverboard_instance = instance;
}

void velCallback(const geometry_msgs::Twist &vel) {
    //hoverboard_instance.sendCommand()
}

void publish_odometry(ros::Publisher hoverboard_odometry, hoverboard_driver::hoverboard_msg feedback) {
    int current_rpm_left = feedback.speedL_meas;
    int current_rpm_right = feedback.speedR_meas;
}

void publishMessage(ros::Publisher odrive_pub, hoverboard_driver::hoverboard_msg msg) {
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

    setInstance(&hoverboard);

    ros::Publisher hoverboard_pub = node.advertise<hoverboard_driver::hoverboard_msg>("hoverboard_msg", 100);

    ros::Publisher hoverboard_odometry = node.advertise<nav_msgs::Odometry>("odometry", 100);

    ros::Subscriber hoverboard_cmd_vel = node.subscribe("cmd_vel", 10, velCallback);

    tf::TransformBroadcaster odom_broadcaster;

    bool hoverboard_error = false;

    while (ros::ok()) {

        bool send_ok = hoverboard.sendCommand(0, 50);

        hoverboard_driver::hoverboard_msg feedback = hoverboard.read_data(&hoverboard_error);

        if (hoverboard_error) {
            ROS_ERROR("Can't connect to hoverboard!");
        } else {
            publishMessage(hoverboard_pub, feedback);
            publish_odometry(hoverboard_odometry, feedback);
        }

        ros::spinOnce();
        rate.sleep();
    }

    hoverboard.close();
}