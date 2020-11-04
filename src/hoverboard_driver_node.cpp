#include <utility>

#include "hoverboard_driver/hoverboard_driver.h"

#define RCV_BUFFER_SIZE 50

#define ODOM_COV 0.005

#define FULL_HEADER 0xABCD
#define HEADER_START 0xCD
#define HEADER_READ_TIMEOUT 1
#define BODY_READ_TIMEOUT 300
#define PACKET_SIZE 29


namespace hoverboard_driver_node {

    class Hoverboard {
    public:
        Hoverboard(std::string serial_name, int baud_rate) : serial_name_(serial_name) {
            serial_ = serial_new();

            if (serial_open(serial_, serial_name.c_str(), baud_rate) < 0) {
                ROS_ERROR("serial_open(): %s\n", serial_errmsg(serial_));
                exit(1);
            }

            last_steer = 0;
            last_speed = 0;
        }

        hoverboard_driver::hoverboard_msg read_data(bool *error) {

            flush_serial_recv_buffer(serial_);

            uint8_t hdr_start_byte = 0xFF;

            hoverboard_driver::hoverboard_msg msg;

            int cnt = 0;

            while (hdr_start_byte != HEADER_START && cnt++ < (PACKET_SIZE - 1)) {
                if (serial_read(serial_, &hdr_start_byte, 1, HEADER_READ_TIMEOUT) < 0) {
                    ROS_ERROR("HDR serial_read");
                    *error = true;
                    return msg;
                };
            }

            if (hdr_start_byte != HEADER_START) {
                ROS_ERROR("HDR read erorr : %i", hdr_start_byte);
                *error = true;
                return msg;
            };

            if (serial_read(serial_, hoverboard_data, PACKET_SIZE, BODY_READ_TIMEOUT) < 0) {
                ROS_ERROR("BODY serial_read");
                *error = true;
                return msg;
            }

            int idx = 1;

            msg.cmd1 = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8);
            msg.cmd2 = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8);
            msg.speedR_meas = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8);
            msg.speedL_meas = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8);
            msg.batVoltage = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8);
            msg.boardTemp = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8);
            msg.cmdLed = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8);

            msg.errorR = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8);
            msg.errorL = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8);

            msg.pulseCountR = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8) + (hoverboard_data[idx++] << 16) +
                              (hoverboard_data[idx++] << 24);
            msg.pulseCountL = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8) + (hoverboard_data[idx++] << 16) +
                              (hoverboard_data[idx++] << 24);

            uint16_t msg_checksum = hoverboard_data[idx++] + (hoverboard_data[idx++] << 8);

            uint16_t calc_checksum =
                    0xABCD ^msg.cmd1 ^msg.cmd2 ^msg.speedR_meas ^msg.speedL_meas ^msg.batVoltage ^msg.boardTemp ^
                    msg.cmdLed;

            if (msg_checksum != calc_checksum) {
                ROS_ERROR("Checksum wrong!: ");
                *error = true;
            } else {
                *error = false;
            }

            return msg;
        };

        bool sendCommand(int16_t steer, int16_t speed) {

            //ROS_DEBUG("Send command : %d - %d", steer, speed);

            uint8_t hoverboard_command[8];

            uint16_t checksum = FULL_HEADER ^steer ^speed;
            int idx = 0;
            hoverboard_command[idx++] = FULL_HEADER & 0xff;
            hoverboard_command[idx++] = (FULL_HEADER >> 8);

            hoverboard_command[idx++] = steer & 0xff;
            hoverboard_command[idx++] = (steer >> 8);

            hoverboard_command[idx++] = speed & 0xff;
            hoverboard_command[idx++] = (speed >> 8);

            hoverboard_command[idx++] = checksum & 0xff;
            hoverboard_command[idx++] = (checksum >> 8);

            if (serial_write(serial_, hoverboard_command, 8) < 0) {
                ROS_ERROR("Write command error!");
                return false;
            };

            serial_flush(serial_);

            return true;
        };

        bool setSteer(int16_t steer) {
            last_steer = steer;
        };

        bool setSpeed(int16_t speed) {
            last_speed = speed;
        };

        bool resendCommand() {
            sendCommand(last_steer, last_speed);
        };

        void close() {
            serial_close(serial_);
            serial_free(serial_);
        };

    private:
        std::string serial_name_;
        serial_t *serial_;
        uint8_t hoverboard_data[RCV_BUFFER_SIZE]{};
        int16_t last_steer;
        int16_t last_speed;
    };

} // namespace hoverboard_driver_node

hoverboard_driver_node::Hoverboard *hoverboard_instance = nullptr;

std::string odom_frame, base_frame;

double base_width;
double wheel_radius;
double wheel_circum;
double rpm_per_meter;
double encoder_cpm;
double coeff;

int encoder_cpr = 90;

//TODO: Odometry class
double raw_wheel_L_ang_pos = std::numeric_limits<double>::max();
double raw_wheel_R_ang_pos = std::numeric_limits<double>::max();

double wheel_L_ang_vel;
double wheel_R_ang_vel;
double wheel_L_ang_pos;
double wheel_R_ang_pos;
double robot_angular_vel;
double robot_angular_pos;
double robot_x_vel;
double robot_y_vel;
double robot_x_pos;
double robot_y_pos;


float global_x;
float global_y;
float global_theta;

void initWheel() {
    wheel_circum = 2.0 * wheel_radius * M_PI;

    coeff = 2.0 * M_PI / encoder_cpr;

    encoder_cpm = encoder_cpr / wheel_circum;

    rpm_per_meter = 1 / wheel_circum;

    ROS_INFO("rpm_per_meter : %f", rpm_per_meter);
}

double getAngularPos(double pulse) {
    return coeff * pulse;
}

void setInstance(hoverboard_driver_node::Hoverboard *instance) {
    hoverboard_instance = instance;
}

void velCallback(const geometry_msgs::Twist &vel) {

    if (hoverboard_instance == nullptr) {
        return;
    }

    double v = vel.linear.x;
    double w = vel.angular.z;

    double rps = v / rpm_per_meter;

    //So it's rpm - rotate per minute
    double rpm = rps * 60;

    if (rpm > 0.0 && rpm < 1.0) {
        rpm = 1;
    } else if (rpm < 0.0 && rpm > -1.0) {
        rpm = -1;
    }

    int16_t speed = static_cast<int>(rpm);
    //TODO: calc
    int16_t steer = static_cast<int>(-1 * w * 30);

    ROS_INFO("Set speed : %d", speed);
    ROS_INFO("Set steer : %d", steer);

    hoverboard_instance->setSteer(steer);
    hoverboard_instance->setSpeed(speed);
}

void
sendOdometry(tf::TransformBroadcaster odom_broadcaster, const ros::Publisher &odometry_pub, ros::Time current_time) {

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_angular_pos);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame;
    odom_trans.child_frame_id = base_frame;

    odom_trans.transform.translation.x = robot_x_pos;
    odom_trans.transform.translation.y = robot_y_pos;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame;

    //set the position
    odom.pose.pose.position.x = robot_x_pos;
    odom.pose.pose.position.y = robot_y_pos;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance[0] = ODOM_COV;
    odom.pose.covariance[7] = ODOM_COV;
    odom.pose.covariance[14] = ODOM_COV;
    odom.pose.covariance[21] = ODOM_COV;
    odom.pose.covariance[28] = ODOM_COV;
    odom.pose.covariance[35] = ODOM_COV;

    //set the velocity
    odom.child_frame_id = base_frame;
    odom.twist.twist.linear.x = robot_x_vel;
    odom.twist.twist.linear.y = robot_y_vel;
    odom.twist.twist.angular.z = robot_angular_vel;

    //publish the message
    odometry_pub.publish(odom);
}

void publishOdometry(hoverboard_driver::hoverboard_msg feedback,
                     const ros::Publisher &odometry_pub,
                     tf::TransformBroadcaster odom_broadcaster,
                     const ros::Time current_time,
                     const ros::Time last_time) {

    double curr_wheel_L_ang_pos = getAngularPos((double) feedback.pulseCountL);
    double curr_wheel_R_ang_pos = getAngularPos((double) feedback.pulseCountR);

    //TODO: fix mess
    if (raw_wheel_L_ang_pos == std::numeric_limits<double>::max() &&
        raw_wheel_R_ang_pos == std::numeric_limits<double>::max()) {
        raw_wheel_L_ang_pos = curr_wheel_L_ang_pos;
        raw_wheel_R_ang_pos = curr_wheel_R_ang_pos;
        return;
    }

    //ROS_INFO("L ang : %f, R ang : %f", curr_wheel_L_ang_pos, curr_wheel_R_ang_pos);

    double dtime = (current_time - last_time).toSec();

    double delta_L_ang_pos = curr_wheel_L_ang_pos - raw_wheel_L_ang_pos;
    double delta_R_ang_pos = -1.0 * (curr_wheel_R_ang_pos - raw_wheel_R_ang_pos);

    delta_L_ang_pos = delta_L_ang_pos;
    delta_R_ang_pos = delta_R_ang_pos;

    raw_wheel_L_ang_pos = curr_wheel_L_ang_pos;
    raw_wheel_R_ang_pos = curr_wheel_R_ang_pos;

    wheel_L_ang_vel = delta_L_ang_pos / (dtime);
    wheel_R_ang_vel = delta_R_ang_pos / (dtime);

    wheel_L_ang_pos += delta_L_ang_pos;
    wheel_R_ang_pos += delta_R_ang_pos;

    robot_angular_vel = (((wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / base_width) - robot_angular_pos) / dtime;
    robot_angular_pos = (wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / base_width;

    robot_x_vel = ((wheel_L_ang_vel * wheel_radius + robot_angular_vel * (base_width / 2.0)) * cos(robot_angular_pos));
    robot_y_vel = ((wheel_L_ang_vel * wheel_radius + robot_angular_vel * (base_width / 2.0)) * sin(robot_angular_pos));

    robot_x_pos = robot_x_pos + robot_x_vel * dtime;
    robot_y_pos = robot_y_pos + robot_y_vel * dtime;

    // send odometry
    sendOdometry(std::move(odom_broadcaster), odometry_pub, current_time);

}

void publishMessage(const ros::Publisher& odrive_pub, hoverboard_driver::hoverboard_msg msg) {
    // Publish message
    odrive_pub.publish(msg);
}

int main(int argc, char **argv) {
    // Start ROS node.
    ROS_INFO("Starting hoverboard_driver node");
    ros::init(argc, argv, "hoverboard_driver");
    ros::NodeHandle node("~");
    ros::Rate rate(30);  // 100 hz

    std::string hoverboard_uart;
    int hoverboard_uart_baudrate;

    node.param<std::string>("odom_frame", odom_frame, "odom");
    node.param<std::string>("base_frame", base_frame, "base_link");

    node.param<std::string>("uart", hoverboard_uart, "/dev/ttyTHS1");
    node.param("baudrate", hoverboard_uart_baudrate, 115200);

    hoverboard_driver_node::Hoverboard hoverboard(hoverboard_uart, hoverboard_uart_baudrate);

    setInstance(&hoverboard);

    node.param<double>("base_width", base_width, 0.43);
    node.param<double>("wheel_radius", wheel_radius, 0.235 / 2);
    node.param("encoder_cpr", encoder_cpr, 90);

    initWheel();

    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();

    ros::Publisher hoverboard_pub = node.advertise<hoverboard_driver::hoverboard_msg>("hoverboard_msg", 20);

    ros::Publisher hoverboard_odometry = node.advertise<nav_msgs::Odometry>("odometry", 20);

    ros::Subscriber hoverboard_cmd_vel = node.subscribe("cmd_vel", 10, velCallback);

    tf::TransformBroadcaster odom_broadcaster;

    bool read_hoverboard_error = false;

    //TODO: replace with rate
    int counter = 0;

    while (ros::ok()) {

        //command hoverboard
        if (counter++ > 5) {
            bool send_ok = hoverboard.resendCommand();
            counter = 0;
        }

        current_time = ros::Time::now();

        hoverboard_driver::hoverboard_msg feedback = hoverboard.read_data(&read_hoverboard_error);

        if (!read_hoverboard_error) {

            publishMessage(
                    hoverboard_pub,
                    feedback
            );

            publishOdometry(
                    feedback,
                    hoverboard_odometry,
                    odom_broadcaster,
                    current_time,
                    last_time);

            last_time = current_time;
        }

        ros::spinOnce();
        rate.sleep();
    }

    hoverboard.close();
}