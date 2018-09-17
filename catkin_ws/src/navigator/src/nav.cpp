#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <amiro_msgs/UInt16MultiArrayStamped.h>
#include "std_msgs/UInt16MultiArray.h"
#include <tf/tf.h>
#include <math.h>
#include "sensor_msgs/Image.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define FLOOR_FRONT_RIGHT 0
#define FLOOR_FRONT_LEFT 3
#define FLOOR_SIDE_RIGHT 1
#define FLOOR_SIDE_LEFT 2
#define X_COORD 0
#define Y_COORD 1
#define X_COORD_MID_MAP 2.50
#define Y_COORD_MID_MAP 2.50
#define accuracyConstPosition 1e-3
#define accuracyConstAngle 1e-6
#define rad2deg 180/M_PI
#define deg2rad M_PI/180
#define fast_rotate 0.5
#define slow_rotate 0.25
#define angular_offset 1e-3
#define angular_correction 0.05
#define x_min_map 0.075
#define x_max_map 4.925
#define y_min_map 0.075
#define y_max_map 3.925
#define beacon_threshold 200
#define high_threshold_gray_100 110
#define low_threshold_gray_100 90
#define high_threshold_gray_125 140
#define low_threshold_gray_125 115
#define high_threshold_gray_150 170
#define low_threshold_gray_150 145
#define black_threshold 50
#define amiro_base_diameter 0.1

/*ros::Publisher*/
ros::Publisher pub;

/*Prototypes*/
void getInitPosition();
void moveToCoordinates(double dest_coords, ros::NodeHandle& n);
void floorProximityCallback(const amiro_msgs::UInt16MultiArrayStamped& msg);
void odometryDataCallback(const nav_msgs::Odometry& msg);
void beaconDetected();

double adjustOrientationForDestination(double dest_coords[]);
double angleCorrection(double dest_angle);
inline bool accurateAngle(double dest_angle, double actual_angle);
inline double accuratePosition(double dest_coord, double actual_coord);
inline double calculateAngle(double x_coord, double y_coord);
inline double calculateAngleOffset(double dest_angle, double actual_angle);


/*Global Struct*/
struct roboData {
    bool beacon_detected;
    size_t proximity_floor_values[4];
    double init_positions[2] = {NULL};
    double odometry_positions[2];
    double odometry_orientation_deg;
} data;

void getInitPosition() {
    bool read_init_positions = false;
    do {
        ros::spinOnce();
        data.init_positions[X_COORD] = data.odometry_positions[X_COORD];
        data.init_positions[Y_COORD] = data.odometry_positions[Y_COORD];
        if((data.init_positions[X_COORD] && data.init_positions[Y_COORD]) != NULL) {
            read_init_positions = true;
        }
    } while(!read_init_positions);
    ROS_INFO("Initial Positions: x:[%f] y:[%f]", data.init_positions[X_COORD], data.init_positions[Y_COORD]);
}

inline bool accurateAngle(double dest_angle, double actual_angle) {
    if (dest_angle >= 0) {
        if((dest_angle - actual_angle) < accuracyConstAngle) return true;
    } else if ((dest_angle - actual_angle) > accuracyConstAngle) return true;
}

inline double accuratePosition(double dest_coord, double actual_coord) {
    ROS_INFO("dest_coord - actual_coord: %f < %f", std::fabs(dest_coord - actual_coord), accuracyConstPosition);
    return std::fabs(dest_coord - actual_coord);
} 

inline double calculateAngle(double x_coord, double y_coord) {
    return atan2(y_coord, x_coord); /*y goes first*/
}

inline double calculateAngleOffset(double dest_angle, double actual_angle) {
    return dest_angle - actual_angle;
}

double adjustOrientationForDestination(double dest_coords[]) {
    geometry_msgs::Twist msg;
    bool acc = false;
    double position_offset_x = dest_coords[X_COORD] - data.odometry_positions[X_COORD];
    double position_offset_y = dest_coords[Y_COORD] - data.odometry_positions[Y_COORD];
    double dest_angle_deg = rad2deg * calculateAngle(position_offset_x, position_offset_y);
    //ROS_INFO("dest_angle: %f", dest_angle_deg);
    ROS_INFO("%f %f %f", position_offset_x, position_offset_y, dest_angle_deg);
    if(dest_angle_deg >= 0 && dest_angle_deg <=180) {
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
    return dest_angle_deg;
}

double angleCorrection(double dest_angle) {
    double angle_offset;
    geometry_msgs::Twist msg;
    angle_offset = calculateAngleOffset(dest_angle, data.odometry_orientation_deg);
    //ROS_INFO("angle_offset: %f", angle_offset);
    if(std::fabs(angle_offset) > angular_offset)
        if(angle_offset <= 0) {
            return -angular_correction;
        } else return angular_correction; 
}

void moveToCoordinates(double *dest_coords) {
    geometry_msgs::Twist msg;
    double acc_x, acc_y;
    double dest_angle_deg = adjustOrientationForDestination(dest_coords);
    //ros::Duration(0.5).sleep();
    msg.linear.x = 0.1;
    do {
        ros::spinOnce();
        acc_x = std::fabs(dest_coords[X_COORD] - data.odometry_positions[X_COORD]);
        acc_y = std::fabs(dest_coords[Y_COORD] - data.odometry_positions[Y_COORD]);
        //acc_x = accuratePosition(dest_coords[X_COORD], data.odometry_positions[X_COORD]);
        //acc_y = accuratePosition(dest_coords[Y_COORD], data.odometry_positions[Y_COORD]);
        msg.angular.z = angleCorrection(dest_angle_deg);
        //if(data.beacon_detected == true) beaconDetected();
        ROS_INFO("dest x - x %f", std::fabs(dest_coords[X_COORD] - data.odometry_positions[X_COORD]));
        ROS_INFO("dest y - y %f", std::fabs(dest_coords[Y_COORD] - data.odometry_positions[Y_COORD]));
        pub.publish(msg);
    } while((acc_x >= accuracyConstPosition) && (acc_y >= accuracyConstPosition)); 
    msg.linear.x = 0;
    pub.publish(msg);
}

void floorProximityCallback(const sensor_msgs::Image::ConstPtr msg0,
              const sensor_msgs::Image::ConstPtr msg1,
              const sensor_msgs::Image::ConstPtr msg2,
              const sensor_msgs::Image::ConstPtr msg3) {

const sensor_msgs::Image::ConstPtr msgs[4] = {msg0, msg1, msg2, msg3};

amiro_msgs::UInt16MultiArrayStamped values;
values.array.data.resize(size(msgs));
values.header = msgs[0]->header;
values.header.frame_id = "";

    for (std::size_t idx = 0; idx < 4; ++idx) {

    // Integrate over all pixel values to get the mean gray value
    size_t grayIntegrated = 0;
        for(auto it = msgs[idx]->data.begin(); it != msgs[idx]->data.end(); ++it) {
            grayIntegrated += size_t(*it);
        }
    // Normalize to 0 .. 255
    const double gray = double(grayIntegrated) / msgs[idx]->data.size();
    data.proximity_floor_values[idx] = gray;
    if(data.proximity_floor_values[idx] < beacon_threshold) data.beacon_detected = true; else data.beacon_detected = false;
    /*ROS_INFO("SIDE LEFT: %zu, FRONT LEFT: %zu, FRONT LEFT: %zu, FRONT RIGHT: %zu",
                                    data.proximity_floor_values[FLOOR_SIDE_LEFT],
                                    data.proximity_floor_values[FLOOR_FRONT_LEFT],
                                    data.proximity_floor_values[FLOOR_FRONT_RIGHT],
                                    data.proximity_floor_values[FLOOR_SIDE_RIGHT]);*/
    }
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
    data.odometry_orientation_deg = y*rad2deg;
    ROS_INFO("x: %f, y: %f, theta: %f", data.odometry_positions[X_COORD],
                                        data.odometry_positions[Y_COORD], 
                                        y*rad2deg);
                                        
}

void beaconDetected() {

}

void moveToMiddleOfTheMap() {
    double middle_of_map[2] = {X_COORD_MID_MAP, Y_COORD_MID_MAP};
    moveToCoordinates(middle_of_map);
}

void checkBorders() {

}

void exploration() {
    double last_hop[2];
    int even_sign = 1, uneven_sign = 1, uneven_step_counter = 1, even_step_counter = 1;
    bool even_step = false;
    double next_hop_x = 0, next_hop_y = 0, last_hop_x = 0, last_hop_y = 0;
    double next_hop_coords[2]{0, 0};
    last_hop_x = data.odometry_positions[X_COORD];
    last_hop_y = data.odometry_positions[Y_COORD];

    //moveToMiddleOfTheMap();
    int i = 0;
    do {  
        if(!even_step) {    
            next_hop_x = last_hop_x + (even_step_counter * amiro_base_diameter * uneven_sign);
            next_hop_y = last_hop_y;
            even_step = true;
            uneven_step_counter++;
            uneven_sign *= (-1);
            next_hop_coords[X_COORD] = next_hop_x;
            next_hop_coords[Y_COORD] = next_hop_y;
            moveToCoordinates(next_hop_coords);
            ROS_INFO("%f, %f", next_hop_x, next_hop_y);
            last_hop_x = next_hop_x;
            last_hop_y = next_hop_y;
        } else {
            next_hop_x = last_hop_x;
            next_hop_y = last_hop_y + (even_step_counter * amiro_base_diameter * even_sign);
            even_step = false;
            even_step_counter++;
            even_sign *= (-1);
            next_hop_coords[X_COORD] = next_hop_x;
            next_hop_coords[Y_COORD] = next_hop_y;
            ROS_INFO("%f, %f", next_hop_x, next_hop_y);
            moveToCoordinates(next_hop_coords);
            last_hop_x = next_hop_x;
            last_hop_y = next_hop_y;
        }
        i++;
    } while(i < 5);
}

main(int argc, char **argv) {
    ros::init(argc, argv, "nav");
    
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("amiro1/cmd_vel", 1);
    ros::Subscriber odometry_sub = n.subscribe("amiro1/odom", 10, odometryDataCallback);
    ros::Rate loop_rate(50);
    geometry_msgs::Twist msg;
    std::string topic_out, topic_in_suffix, topic_in_prefix;

    n.param<std::string>("topic_in_suffix", topic_in_suffix, "/amiro1/proximity_floor_");
    n.param<std::string>("topic_in_prefix", topic_in_prefix, "/image_raw");
    n.param<std::string>("topic_out", topic_out, "/amiro1/proximity_floor/values");

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image> syncPolicy;

    message_filters::Subscriber<sensor_msgs::Image> sub0(n, topic_in_suffix + "0" + topic_in_prefix, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub1(n, topic_in_suffix + "1" + topic_in_prefix, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub2(n, topic_in_suffix + "2" + topic_in_prefix, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub3(n, topic_in_suffix + "3" + topic_in_prefix, 1);

    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub0, sub1, sub2, sub3);
    sync.registerCallback(boost::bind(&floorProximityCallback, _1, _2, _3, _4));

    getInitPosition();
    double dest_coords[2] = {2.5, 2.6};
    ros::Duration(3.0).sleep();
    //generateRandomCoords();
    //adjustOrientationForDestination(dest_coords);
    moveToCoordinates(dest_coords);
    //exploration();
    /*while(ros::ok()) {
        //msg.linear.x = 0.1;
        //pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }*/
    ros::shutdown();
    return 0;
}
