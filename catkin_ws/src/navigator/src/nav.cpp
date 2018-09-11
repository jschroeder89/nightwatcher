#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <amiro_msgs/UInt16MultiArrayStamped.h>
#include "std_msgs/UInt16MultiArray.h"
#include <tf/tf.h>
#include <math.h>

#define FLOOR_FRONT_RIGHT 0
#define FLOOR_FRONT_LEFT 3
#define FLOOR_SIDE_RIGHT 1
#define FLOOR_SIDE_LEFT 2
#define X_COORD 0
#define Y_COORD 1
#define accuracyConst 1e-6
#define rad2deg 180/M_PI
#define deg2rad M_PI/180
#define fast_rotate 0.5
#define slow_rotate 0.25

/*ros::Publisher*/
ros::Publisher pub;

/*Prototypes*/
void getInitPosition();
void moveToCoordinates(double dest_coords, ros::NodeHandle& n);
void floorProximityCallback(const amiro_msgs::UInt16MultiArrayStamped& msg);
void odometryDataCallback(const nav_msgs::Odometry& msg);
double adjustOrientationForDestination(double *dest_coords);
inline bool accuratePosition(double dest_coord, double actual_coord);
inline bool accurateAngle(double dest_angle, double actual_angle);
inline double calculateAngle(double x_coord, double y_coord);

/*Global Struct*/
struct roboData {
    double init_positions[2] = {NULL};
    double odometry_positions[2];
    double odometry_orientation_rad;
    double odometry_orientation_deg;
    double traveled_distance[2];
    double random_dest_coords[2];
    std_msgs::UInt16MultiArray floor_values;
} data;

void getInitPosition() {
    bool read_init_positions = false;
    do {
        ros::spinOnce();
        data.init_positions[X_COORD] = data.odometry_positions[X_COORD];
        data.init_positions[Y_COORD] = data.odometry_positions[Y_COORD];
        if((data.init_positions[X_COORD] && data.init_positions) != NULL) {
            read_init_positions = true;
        }
    } while(!read_init_positions);
    ROS_INFO("Initial Positions: x:[%f] y:[%f]", data.init_positions[X_COORD], data.init_positions[Y_COORD]);
}

void generateRandomCoords() {

}

inline bool accuratePosition(double dest_coord, double actual_coord) {
    if(std::fabs((dest_coord - actual_coord) < accuracyConst)) {
        return true;
    } else return false; 
} 

inline bool accurateAngle(double dest_angle, double actual_angle) {
    if(std::fabs((dest_angle - actual_angle) < accuracyConst)){
        return true;
    } else return false; 
}

inline double calculateAngle(double x_coord, double y_coord) {
    return atan2(y_coord, x_coord); /*y goes first*/
}

double adjustOrientationForDestination(double *dest_coords) {
    geometry_msgs::Twist msg;
    bool acc = false;
    double position_offset_x = dest_coords[X_COORD] - data.odometry_positions[X_COORD];
    double position_offset_y = dest_coords[Y_COORD] - data.odometry_positions[Y_COORD];
    double dest_angle_deg = rad2deg * calculateAngle(position_offset_x, position_offset_y);
    if(dest_angle_deg <= 180) {
        msg.angular.z = fast_rotate;
    } else msg.angular.z = -fast_rotate;
    do {
        ros::spinOnce();
        acc = accurateAngle(dest_angle_deg, data.odometry_orientation_deg);
        pub.publish(msg);
    } while(!acc && ros::ok());
    msg.angular.z = 0;
    pub.publish(msg);
    ros::spinOnce();
}

void moveToCoordinates(double *dest_coords) {
    geometry_msgs::Twist msg;
    bool acc = false;
    msg.linear.x = 0.1;
    do {
        for(size_t i = 0; i < 2; i++) {
            ros::spinOnce();
            acc = accuratePosition(dest_coords[i], data.odometry_positions[i]);
        }
        pub.publish(msg);
    } while(!acc && ros::ok()); 
    msg.linear.x = 0;
    pub.publish(msg);
    ros::spinOnce();
}

void floorProximityCallback(const amiro_msgs::UInt16MultiArrayStamped& msg) {
    data.floor_values.data = msg.array.data;
    /*ROS_INFO("\nFLOOR_FRONT_LEFT: %d \nFLOOR_RIGHT_RIGHT: %d \nFLOOR_SIDE_LEFT: %d \nFLOOR_SIDE_RIGHT: %d", 
              data.floor_values.data[FLOOR_FRONT_LEFT],
              data.floor_values.data[FLOOR_FRONT_RIGHT],
              data.floor_values.data[FLOOR_SIDE_LEFT],
              data.floor_values.data[FLOOR_SIDE_RIGHT]);*/
}

void odometryDataCallback(const nav_msgs::Odometry& msg) {
    data.odometry_positions[X_COORD] = msg.pose.pose.position.x;
    data.odometry_positions[Y_COORD] = msg.pose.pose.position.y;
    tf::Quaternion q(msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    if (y*rad2deg < 0) {
        data.odometry_orientation_deg = 180.00 + (180.00 - std::fabs(y)*rad2deg);
    } else data.odometry_orientation_deg = y*rad2deg;    
    ROS_INFO("x: %f, y: %f, theta: %f", data.odometry_positions[X_COORD],
                                        data.odometry_positions[Y_COORD], 
                                        data.odometry_orientation_deg);
}

main(int argc, char **argv) {
    ros::init(argc, argv, "nav");
    
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("amiro1/cmd_vel", 1);
    ros::Subscriber floorProximity_sub = n.subscribe("amiro1/proximity_floor/values", 10, floorProximityCallback);
    ros::Subscriber odometry_sub = n.subscribe("amiro1/odom", 10, odometryDataCallback);
    ros::Rate loop_rate(50);
    geometry_msgs::Twist msg;

    getInitPosition();

    ros::Duration(3.0).sleep();
    while(ros::ok()) {
        generateRandomCoords();
        adjustOrientationForDestination(data.random_dest_coords);
        moveToCoordinates(data.random_dest_coords);
        //msg.angular.z = 0.1;
        //pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
