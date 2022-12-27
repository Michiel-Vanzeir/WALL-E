#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <wiringPi.h>

#include <nav_msgs/Odometry.h>

#define RIGHT_ENCODER 7
#define LEFT_ENCODER 1
#define WHEEL_RADIUS 4.0 // cm

double left_rotation = 0;
double right_rotation = 0;

bool inForwardMode(int pin) {
    if (pin == 7) {
        return digitalRead(11);
    } else if (pin == 1) {
        return digitalRead(2);
    }
}

void rightEncoderCallback() {
    ROS_INFO("Right encoder callback");
    if (inForwardMode(RIGHT_ENCODER)) {
        right_rotation += 1/20.0 * (WHEEL_RADIUS*2*M_PI);
    } else {
        right_rotation -= 1/20.0 * (WHEEL_RADIUS*2*M_PI);
    }
}

void leftEncoderCallback() {
    ROS_INFO("Left encoder callback");
    if (inForwardMode(LEFT_ENCODER)) {
        left_rotation += 1/20.0 * (WHEEL_RADIUS*2*M_PI);
    } else {
        left_rotation -= 1/20.0 * (WHEEL_RADIUS*2*M_PI);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_advertiser");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 2);
    tf::TransformBroadcaster odom_broadcaster;

    // Setup wiringpi so that it uses the BOARD pin numbering scheme of a raspberry pi
    wiringPiSetup();
    pinMode(RIGHT_ENCODER, INPUT);
    pinMode(LEFT_ENCODER, INPUT);

    wiringPiISR(RIGHT_ENCODER, INT_EDGE_BOTH, &rightEncoderCallback);
    wiringPiISR(LEFT_ENCODER, INT_EDGE_BOTH, &leftEncoderCallback);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    double prev_left_rotation = 0;
    double prev_right_rotation = 0;

    ros::Rate r(10.0);
    while (nh.ok()) {
        ros::spinOnce(); 
        current_time = ros::Time::now();
        ROS_INFO("Left rotation: %f", left_rotation);
        // Calculate the velocity of the rotation
        double dt = (current_time - last_time).toSec();
        double left_velocity = (left_rotation - prev_left_rotation) / dt;
        double right_velocity = (right_rotation - prev_right_rotation) / dt;

        // Broadcast the tf transform
        double left_roll = left_rotation / (WHEEL_RADIUS*2*M_PI);
        double right_roll = right_rotation / (WHEEL_RADIUS*2*M_PI);
        geometry_msgs::Quaternion odom_quat_left = tf::createQuaternionMsgFromRollPitchYaw(left_roll, 0, 0);
        geometry_msgs::Quaternion odom_quat_right = tf::createQuaternionMsgFromRollPitchYaw(right_roll, 0, 0);

        geometry_msgs::TransformStamped left_wheel_transform;
        left_wheel_transform.header.stamp = current_time;
        left_wheel_transform.header.frame_id = "odom";
        left_wheel_transform.child_frame_id = "left_wheel";
        left_wheel_transform.transform.translation.x = left_rotation;
        left_wheel_transform.transform.rotation = odom_quat_left;

        geometry_msgs::TransformStamped right_wheel_transform;
        right_wheel_transform.header.stamp = current_time;
        right_wheel_transform.header.frame_id = "odom";
        right_wheel_transform.child_frame_id = "right_wheel";
        right_wheel_transform.transform.translation.x = right_rotation;
        right_wheel_transform.transform.rotation = odom_quat_right;

        odom_broadcaster.sendTransform(left_wheel_transform);
        odom_broadcaster.sendTransform(right_wheel_transform);

        // Publish the odometry msg
        nav_msgs::Odometry left_odom, right_odom;
        left_odom.header.stamp = current_time;
        left_odom.header.frame_id = "odom";
        left_odom.child_frame_id = "left_wheel";
        left_odom.pose.pose.position.x = left_rotation;
        left_odom.twist.twist.linear.x = left_velocity;

        right_odom.header.stamp = current_time;
        right_odom.header.frame_id = "odom";
        right_odom.child_frame_id = "right_wheel";
        right_odom.pose.pose.position.x = right_rotation;
        right_odom.twist.twist.linear.x = right_velocity;

        odom_pub.publish(left_odom);
        odom_pub.publish(right_odom);

        // Update the values for the next iteration
        prev_left_rotation = left_rotation;
        prev_right_rotation = right_rotation;
        last_time = current_time;

        r.sleep();
    }
}
