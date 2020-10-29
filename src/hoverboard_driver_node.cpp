#include "hoverboard_driver/hoverboard_driver.h"

#define HEADER_READ_TIMEOUT 50
#define BODY_READ_TIMEOUT 100

#define RCV_BUFFER_SIZE 50

namespace hoverboard_driver_node {

    class Hoverboard {
    public:
        Hoverboard(std::string serial_name) : serial_name_(serial_name) {
            serial_ = serial_new();

            if (serial_open(serial_, serial_name.c_str(), 115200) < 0) {
                ROS_ERROR("serial_open(): %s\n", serial_errmsg(serial_));
                exit(1);
            }

            //flush_serial_recv_buffer(serial_);

            //buffer_size = 50;

            //serial_input_waiting(serial_, &buffer_size);

            last_steer = 0;
            last_speed = 0;
        }

        hoverboard_driver::hoverboard_msg read_data(bool *error) {

            //std::memset(hoverboard_data, 0, sizeof hoverboard_data);
            //hoverboard_data[0] = 0xFF;
            uint8_t hdr_start_byte = 0xFF:

            hoverboard_driver::hoverboard_msg msg;

            if (serial_read(serial_, hdr_start_byte, 1, HEADER_READ_TIMEOUT) < 0) {
                ROS_ERROR("serial_read");
                *error = true;
                return msg;
            };

            if (hdr_start_byte != 0xCD) {
                ROS_ERROR("HDR : %i", hoverboard_data[0]);
                *error = true;
                return msg;
            };

            if (serial_read(serial_, hoverboard_data, 29, BODY_READ_TIMEOUT) < 0) {
                ROS_ERROR("Body serial_read");
                *error = true;
                return msg;
            }

            int idx = 1;

            msg.cmd1 = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);
            msg.cmd2 = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);
            msg.speedR_meas = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);
            msg.speedL_meas = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);
            msg.batVoltage = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);
            msg.boardTemp = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);
            msg.cmdLed = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);

            msg.errorR = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);
            msg.errorL = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);

            msg.pulseCountR = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8) + (hoverboard_data[idx++]<< 16) + (hoverboard_data[idx++]<< 24);
            msg.pulseCountL = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8) + (hoverboard_data[idx++]<< 16) + (hoverboard_data[idx++]<< 24);

            uint16_t msg_checksum = hoverboard_data[idx++] + (hoverboard_data[idx++]<< 8);

            uint16_t calc_checksum = 0xABCD ^ msg.cmd1 ^ msg.cmd2 ^ msg.speedR_meas ^ msg.speedL_meas ^ msg.batVoltage ^ msg.boardTemp ^ msg.cmdLed;

            if(msg_checksum != calc_checksum){
                ROS_ERROR("Checksum wrong!: ");
                *error = true;
            } else {
                *error = false;
            }

            return msg;
        };

        bool sendCommand(int16_t steer, int16_t speed){

            //ROS_DEBUG("Send command : %d - %d", steer, speed);

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

            serial_flush(serial_);

            return true;
        };

        bool setSteer(int16_t steer){
            last_steer = steer;
        };

        bool setSpeed(int16_t speed){
            last_speed = speed;
        };

        bool resendCommand(){
            sendCommand(last_steer, last_speed);
        };

        void close() {
            serial_close(serial_);
            serial_free(serial_);
        };

    private:
        std::string serial_name_;
        serial_t *serial_;
        uint8_t hoverboard_data[RCV_BUFFER_SIZE];
        int16_t last_steer;
        int16_t last_speed;
        //TODO : param
        unsigned int buffer_size;
    };

} // namespace hoverboard_driver_node

hoverboard_driver_node::Hoverboard *hoverboard_instance = NULL;

float base_width;
float wheel_radius;
float wheel_circum;
float rpm_per_meter;

int right_pos = 0;
int left_pos = 0;

ros::Time current_time, last_time;

float global_x;
float global_y;
float global_theta;

void setInstance(hoverboard_driver_node::Hoverboard *instance){
    hoverboard_instance = instance;
}

void velCallback(const geometry_msgs::Twist &vel) {

    if(hoverboard_instance == NULL){
        return;
    }

    float v = vel.linear.x;
    float w = vel.angular.z;

    float rps = v / rpm_per_meter;

    //So it's rpm - rotate per minute
    float rpm = rps * 60;

    if(rpm > 0.0 && rpm < 1.0){
        rpm = 1;
    } else
    if(rpm  < 0.0 && rpm > -1.0){
        rpm = -1;
    }

    int16_t speed = static_cast<int>(rpm);
    //TODO: calc
    int16_t steer = static_cast<int>(-1 * w * 50);

    ROS_INFO("Set speed : %d", speed);
    ROS_INFO("Set steer : %d", steer);

    hoverboard_instance->setSteer(steer);
    hoverboard_instance->setSpeed(speed);
}

void publishOdometry(ros::Publisher odometry_pub,
                      hoverboard_driver::hoverboard_msg feedback,
                      tf::TransformBroadcaster odom_broadcaster,
                      const ros::Time current_time,
                      const ros::Time last_time) {
    ros::Duration ros_time_elapsed = current_time - last_time;
    float time_elapsed = ros_time_elapsed.toSec();

    // Forward
    // speedR_meas - negative
    // speedL_meas - positive
    int current_feedback_left = feedback.speedL_meas;
    int current_feedback_right = -1 * feedback.speedR_meas;

    //Assume feedback is rpm
    //rpm to meters
    float current_rps_left = current_feedback_left / 60.0;
    float current_rps_right = current_feedback_right / 60.0;

    float delta_right_wheel_in_meter = current_rps_left * time_elapsed;
    float delta_left_wheel_in_meter = current_rps_right * time_elapsed;

    float local_theta = (delta_right_wheel_in_meter - delta_left_wheel_in_meter) / base_width;

    float distance = (delta_right_wheel_in_meter + delta_left_wheel_in_meter) / 2;

    float local_x = cos(global_theta) * distance;
    float local_y = -sin(global_theta) * distance;

    global_x = global_x + (cos(global_theta) * local_x - sin(global_theta) * local_y);
    global_y = global_y + (sin(global_theta) * local_x + cos(global_theta) * local_y);

    global_theta += local_theta;

    //global_theta = math.atan2(math.sin(global_theta), math.cos(global_theta));

    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, global_theta);

    ros::Time now_time = ros::Time::now();

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(global_x, global_y, 0.0));
    transform.setRotation(quaternion);

    odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "odom"));

    nav_msgs::Odometry odom;
    odom.header.stamp = now_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = global_x;
    odom.pose.pose.position.y = global_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(global_theta);
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = distance / time_elapsed;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = local_theta / time_elapsed;
    odometry_pub.publish(odom);


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
    ros::Rate rate(100);  // 100 hz

    std::string hoverboard_uart;

    node.param<std::string>("hoverboard_uart", hoverboard_uart, "/dev/ttyTHS1");

    hoverboard_driver_node::Hoverboard hoverboard(hoverboard_uart);

    setInstance(&hoverboard);

    node.param<float>("base_width", base_width, 0.43);
    node.param<float>("wheel_radius", wheel_radius, 0.235 / 2);

    wheel_circum = 2.0 * wheel_radius * M_PI;

    rpm_per_meter = 1 / wheel_circum;

    ROS_INFO("rpm_per_meter : %f", rpm_per_meter);

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Publisher hoverboard_pub = node.advertise<hoverboard_driver::hoverboard_msg>("hoverboard_msg", 20);

    ros::Publisher hoverboard_odometry = node.advertise<nav_msgs::Odometry>("odometry", 20);

    ros::Subscriber hoverboard_cmd_vel = node.subscribe("cmd_vel", 10, velCallback);

    tf::TransformBroadcaster odom_broadcaster;

    bool read_hoverboard_error = false;

    //TODO: replace with rate
    int counter = 0;

    while (ros::ok()) {

        //command hoverboard
        if(counter++ > 5) {
            //bool send_ok = hoverboard.resendCommand();
            counter = 0;
        }

        hoverboard_driver::hoverboard_msg feedback = hoverboard.read_data(&read_hoverboard_error);

        if (!read_hoverboard_error) {
            publishMessage(hoverboard_pub, feedback);
            //publishOdometry(hoverboard_odometry, feedback, odom_broadcaster, current_time, last_time);
        }

        ros::spinOnce();
        rate.sleep();
    }

    hoverboard.close();
}