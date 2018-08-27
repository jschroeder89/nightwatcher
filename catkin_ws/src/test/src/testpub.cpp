#include "ros/ros.h"
#include <geometry_msgs/Twist.h>



int main(int argc, char **argv) {
    ros::init(argc, argv, "velpub");

    ros::NodeHandle node_handle;
    ros::Publisher pub = node_handle.advertise<geometry_msgs::Twist>("amiro1/cmd_vel",1000);

    ros::Rate loop_rate(10);
    
    geometry_msgs::Twist msg;
    while(ros::ok()){
        msg.linear.x = 0.005;
        pub.publish(msg);
        ROS_INFO("%f", msg.linear.x);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
