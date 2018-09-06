#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <amiro_msgs/UInt16MultiArrayStamped.h>
#include "std_msgs/UInt16MultiArray.h"


ros::Publisher pub;
int floor_values[4];


void floorProximityCallback(const amiro_msgs::UInt16MultiArrayStamped& msg) {
    
    for(size_t i = 0; i < 3; i++) {
        floor_values[i] = msg.array.data[i];    
        ROS_INFO("%d", floor_values[i]);
    }
    
}

main(int argc, char **argv) {
    ros::init(argc, argv, "nav");
    
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("amiro1/cmd_vel", 1);
    ros::Subscriber floorProximity_sub = n.subscribe("amiro1/proximity_floor/values", 10, floorProximityCallback);
    ros::Rate loop_rate(50);

    geometry_msgs::Twist msg;
    while(ros::ok()) {
        msg.linear.x = 0.05;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
