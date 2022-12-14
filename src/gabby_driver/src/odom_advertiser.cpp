#include <chrono>
#include <ros/ros.h>
#include <wiringPi.h>

#include <nav_msgs/Odometry.h>

constexpr int LEFT_ENCODER = 11;
constexpr int RIGHT_ENCODER = 13;

long coder[2] = {0, 0};
int lastSpeed[2] = {0, 0};

uint32_t getMillis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}   

void leftWheelSpeed() {
    coder[0]++;
}

void rightWheelSpeed() {
    coder[1]++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_advertiser");
    ros::NodeHandle nh;
    
    // Configure the encoder's GPIO pins
    wiringPiSetup();
    pinMode(LEFT_ENCODER, INPUT);
    pinMode(RIGHT_ENCODER, INPUT);
    wiringPiISR(LEFT_ENCODER, INT_EDGE_RISING, &leftWheelSpeed);
    wiringPiISR(RIGHT_ENCODER, INT_EDGE_RISING, &rightWheelSpeed);

    ros::Publisher left_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 2);
    ros::Publisher right_odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 2);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        nav_msgs::Odometry left_odom;
        nav_msgs::Odometry right_odom;

        left_odom.header.stamp = ros::Time::now();
        right_odom.header.stamp = ros::Time::now();

        left_odom.child_frame_id = "left_wheel";
        right_odom.child_frame_id = "right_wheel";

        left_odom.twist.twist.linear.x = coder[0] - lastSpeed[0];
        right_odom.twist.twist.linear.x = coder[1] - lastSpeed[1];

        lastSpeed[0] = coder[0];
        lastSpeed[1] = coder[1];
        coder[0] = 0;
        coder[1] = 0;
    }

    ros::spinOnce();
    return 0;
}