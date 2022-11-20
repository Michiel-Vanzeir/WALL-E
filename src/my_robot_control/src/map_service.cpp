#include "ros/ros.h"

mapCallback()

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_service");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("map_service", mapCallback);
    
    ros::spin();
    return 0;
}