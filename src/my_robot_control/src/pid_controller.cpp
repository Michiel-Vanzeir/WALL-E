#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "my_robot_msgs/Throttle.h"
#include "my_robot_msgs/Inputvars.h"

ros::Publisher throttlepub;
int _integral = 0;
int _prev_error = 0;
int integral_counter = 0;

void inputCallback(const my_robot_msgs::Inputvars::ConstPtr& msg) {
    int error = msg->error;
    double std_throttle = 0.25;

    double _Kp = 0.0009;
    double _Ki = 0.0000003;
    double _Kd = 0.005;

    // Make sure the integral parameter doesn't have a too large influence over time
    _integral += error;
    integral_counter++;
    if (count >= 20) {
        _integral = 0;
        integral_counter = 0;
    }

    double output = _Kp*error + _Ki*_integral + _Kd*(error - _prev_error);

    auto message = my_robot_msgs::Throttle();
    message.left_throttle = std_throttle + output;
    message.right_throttle = std_throttle - output;

    throttlepub.publish(message);

    _prev_error = error;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pid_controller");
    ros::NodeHandle nh;

    throttlepub = nh.advertise<my_robot_msgs::Throttle>("throttlefeed", 2);
    ros::Subscriber sub = nh.subscribe("inputfeed", 2, inputCallback);

    ros::spin();
}
