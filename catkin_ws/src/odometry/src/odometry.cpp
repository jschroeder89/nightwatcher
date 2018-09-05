#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>



main(int argc, char **argv) {
    ros::init(argc, argv, "odotest");

    ros::NodeHandle nh;

    ros::Publisher od = nh.advertise<nav_msgs::Odometry>("amiro1/odom", 1000);
    //ros::Publisher nav = nh.advertise<geometry_msgs::Twist>("amiro1/cmd_vel", 1000);

    ros::Rate loop_rate(10);

    //geomtery_msgs::Twist vel_msg;
    nav_msgs::Odometry odo_msg;
    odo_msg.pose.pose.position.x = 0.1;
    while(ros::ok()) {
        //vel_mgs.linear.x = 0.1;
        //nav.pubilsh(vel_mgs);
        //od.publish(odo_msg.pose.pose.position.x);
        //ROS_INFO("%f", odo_msg.pose.pose.position.x);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
