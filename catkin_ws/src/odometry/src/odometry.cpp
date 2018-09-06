#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>

void odomCallback(const nav_msgs::Odometry& msg) {
    ROS_INFO("x:[%f]\n y:[%f]\n theta:[%f]\n", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.x);
}

main(int argc, char **argv) {
    ros::init(argc, argv, "odometry");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("amiro1/odom", 1000, odomCallback);
    ros::spin();
    
    return 0;
}
