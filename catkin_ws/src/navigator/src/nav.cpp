#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <amiro_msgs/UInt16MultiArrayStamped.h>
#include "std_msgs/UInt16MultiArray.h"



struct roboData {
    float odometry_positions[2];
    float odometry_orientation;
    float traveled_distance[2];
    std_msgs::UInt16MultiArray floor_values;
};

roboData data;
ros::Publisher pub;


void floorProximityCallback(const amiro_msgs::UInt16MultiArrayStamped& msg) {
    data.floor_values.data = msg.array.data;
    for(size_t i = 0; i < 3; i++) {
        ROS_INFO("%d", data.floor_values.data[i]);
    }
    
}

void odometryDataCallback(const nav_msgs::Odometry& msg) {

}

main(int argc, char **argv) {
    ros::init(argc, argv, "nav");
    
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("amiro1/cmd_vel", 1);
    ros::Subscriber floorProximity_sub = n.subscribe("amiro1/proximity_floor/values", 10, floorProximityCallback);
    ros::Subscriber odometry_sub = n.subscribe("amiro1/odom", 10, odometryDataCallback);
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
