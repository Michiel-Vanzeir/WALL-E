#include <ros/ros.h>
#include <wiringPi.h>

#include "gabby_msgs/EncoderData.h"

#define RIGHT_ENCODER 7
#define LEFT_ENCODER 1
#define WHEEL_RADIUS 0.040 // meters

double left_distance = 0;
double right_distance = 0;

bool inForwardMode(int pin) {
    if (pin == RIGHT_ENCODER) {
        return digitalRead(11);
    } else if (pin == LEFT_ENCODER) {
        return digitalRead(2);
    }
}

void rightEncoderCallback() {
    if (inForwardMode(RIGHT_ENCODER)) {
        right_distance += 1/20.0 * (2 * M_PI * WHEEL_RADIUS);
    } else {
        right_distance -= 1/20.0 * (2 * M_PI * WHEEL_RADIUS);
    }
}

void leftEncoderCallback() {
    if (inForwardMode(LEFT_ENCODER)) {
        left_distance += 1/20.0 * (2 * M_PI * WHEEL_RADIUS);
    } else {
        left_distance -= 1/20.0 * (2 * M_PI * WHEEL_RADIUS);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheel_odom_advertiser");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<gabby_msgs::EncoderData>("/wheel_odom", 2);

    // Setup wiringpi so that it uses the BOARD pin numbering scheme of a raspberry pi
    wiringPiSetup();
    pinMode(RIGHT_ENCODER, INPUT);
    pinMode(LEFT_ENCODER, INPUT);

    wiringPiISR(RIGHT_ENCODER, INT_EDGE_BOTH, &rightEncoderCallback);
    wiringPiISR(LEFT_ENCODER, INT_EDGE_BOTH, &leftEncoderCallback);

    ros::Rate r(10.0);
    while (nh.ok()) {
        ros::spinOnce();

        gabby_msgs::EncoderData msg;
        msg.left_distance = left_distance;
        msg.right_distance = right_distance;

        odom_pub.publish(msg);
        r.sleep();
    }
}
