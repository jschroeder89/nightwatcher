#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

void velCallback(const geometry_msgs::Twist& msg) {
    //ROS_INFO("Vel: [%f]", msg.linear.x);
}

main(int argc, char **argv)
{
    ros::init(argc, argv, "testsub");
    ros::NodeHandle nh;

    ros::Subscriber testsub = nh.subscribe("amiro1/cmd_vel", 1000, velCallback);
    ros::spin();
    return 0;
}
