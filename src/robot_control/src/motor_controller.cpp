#include "ros/ros.h"
#include "robot_control/motor_cmd.h"
#include "std_msgs/String.h"

void motor_command(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received the following command: [%s]", msg->data.c_str());
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;
    ros::Subscriber motor_sub = n.subscribe("motor_controls", 100, motor_command);
    ros::spin();    
};
