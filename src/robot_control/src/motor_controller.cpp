#include "ros/ros.h"
#include "robot_control/motor_cmd.h"

void motor_command(const robot_control::motor_cmd::ConstPtr& msg) {
    ROS_INFO("I heard something [%d]", msg->left_motor);
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;
    ros::Subscriber motor_sub = n.subscribe("motor_controls", 100, motor_command);
};
