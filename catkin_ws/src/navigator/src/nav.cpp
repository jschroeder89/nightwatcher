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
#include "yaml-cpp/yaml.h"
#include <fstream>

#define FRONT_RIGHT 0
#define FRONT_LEFT 3
#define SIDE_RIGHT 1
#define SIDE_LEFT 2
#define X_COORD 0
#define Y_COORD 1
#define THETA 2
#define X_COORD_MID_MAP 2.50
#define Y_COORD_MID_MAP 2.50
#define accuracyConstPosition 0.005
#define accuracyConstAngle 0.001
#define rad2deg 180/M_PI
#define deg2rad M_PI/180
#define fast_rotate_ccw 0.3
#define fast_rotate_cw -0.3
#define slow_rotate_ccw 0.25
#define slow_rotate_cw -0.25
#define holy_fuck_this_is_slow_ccw 0.07
#define holy_fuck_this_is_slow_cw -0.07
#define angular_offset 1e-6
#define angular_correction 0.025
#define x_min_map 0.075
#define x_max_map 4.925
#define y_min_map 0.075
#define y_max_map 3.925
#define beacon_threshold 200
#define areal_threshold 0.2
#define high_threshold_gray_100 110
#define low_threshold_gray_100 90
#define high_threshold_gray_125 140
#define low_threshold_gray_125 115
#define high_threshold_gray_150 170
#define low_threshold_gray_150 145
#define black_threshold 50
#define amiro_base_diameter 0.1
#define quadrant_I 1
#define quadrant_II 2
#define quadrant_III 3
#define quadrant_VI 4

/*ros::Publisher*/
ros::Publisher pub;

/*YAML::Emitter*/
YAML::Emitter out;

struct beacon {
    int id;
    double pose2d[3];
    std::string name;
};

beacon base; 
beacon beacon_1; 
beacon beacon_2; 
beacon beacon_3; 
beacon beacon_4; 
beacon beacon_5; 
beacon beacon_6; 
beacon beacon_array[7];

// NOTE Zielpfad der .yaml-Datei, Bitte vor dem Testen ändern!
std::string yaml_path = "/homes/joschroeder/route.yaml"; 

/*Global Struct*/
struct roboData {
    bool beacon_detected;
    int detecting_sensor_id;
    int beacon_counter = 0;
    size_t proximity_floor_values[4];
    double init_positions[2] = {0, 0};
    double odometry_positions[2];
    double odometry_orientation_deg;
    double odometry_orientation_beacon_first_contact;
    double odometry_orientation_beacon_second_contact;
} data;

/*Prototypes*/
int getQuadrant(double angle);
int getBeaconId();
bool scannedBeacon();
bool proximityFrontLeftDetected();
bool proximityFrontRightDetected();
bool proximitySideLeftDetected();
bool proximitySideRightDetected();
bool proximityFrontBlack();
bool proximityAllDetected();
void appendYAML(beacon &b);
void setAngularVelocity(double val);
void setLinearVelocity(double val);
void getInitPosition();
void moveToCoordinates(double *dest_coords);
void rotateToDestination(double dest_angle, double actual_angle);
void floorProximityCallback(const amiro_msgs::UInt16MultiArrayStamped& msg);
void odometryDataCallback(const nav_msgs::Odometry& msg);
void beaconDetected(double dest_coords[]);
void exploration();
double adjustOrientationForDestination(double dest_coords[]);
double angleCorrection(double dest_angle, double actual_angle);
inline double calculateAngle(double x_coord, double y_coord);


void getInitPosition() {
    bool read_init_positions = false;
    do {
        ros::spinOnce();
        data.init_positions[X_COORD] = data.odometry_positions[X_COORD];
        data.init_positions[Y_COORD] = data.odometry_positions[Y_COORD];
        if ((data.init_positions[X_COORD] && data.init_positions[Y_COORD]) != 0) {
            read_init_positions = true;
        }
    } while (!read_init_positions);
    ROS_INFO("Initial Positions: x:[%f] y:[%f]", data.init_positions[X_COORD], data.init_positions[Y_COORD]);
}

inline double calculateAngle(double x_coord, double y_coord) {
    return atan2(y_coord, x_coord); /*y goes first*/
}

int getQuadrant(double angle) {
    if (angle <= 90 && angle >= 0) {
        return quadrant_I;
    } else if(angle <= 180 && angle > 90) {
        return quadrant_II;
    } else if(angle <= 270 && angle > 180) {
        return quadrant_III;
    } else if(angle <= 360 && angle > 270) {
        return quadrant_VI;
    } else return 0;
}

void rotateToDestination(double dest_angle, double actual_angle) {
    int dest_quadrant = getQuadrant(dest_angle);
    int actual_quadrant = getQuadrant(actual_angle);
    ROS_INFO("dest_quadrant: %i", dest_quadrant);
    ROS_INFO("act_quadrant: %i", actual_quadrant);
    double deg_diff = 0;
    int q = 0;
    geometry_msgs::Twist msg;

    /*Destination Angle and Actual Angle in same Quadrant*/
    if(dest_quadrant == actual_quadrant) {
        if(dest_angle > actual_angle) {
            msg.angular.z = fast_rotate_ccw; 
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                //ROS_INFO("deg_diff: %f", deg_diff);
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff > accuracyConstAngle);
        } else {
            msg.angular.z = fast_rotate_cw;
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff < accuracyConstAngle); 
        }
    } 

    /*Destination Angle in Quadrant I*/
    else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_II) {
        msg.angular.z = fast_rotate_cw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff > accuracyConstAngle);
    } else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_VI) {
        msg.angular.z = fast_rotate_ccw;
        q = 0;
        do {
            q = getQuadrant(data.odometry_orientation_deg);
            pub.publish(msg);
            ros::spinOnce(); 
        } while(q != quadrant_I);
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff > accuracyConstAngle);
    } else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_III) {
        if((actual_angle - dest_angle) <= 180) {
            msg.angular.z = fast_rotate_ccw; 
            q = 0;
            do {
                q = getQuadrant(data.odometry_orientation_deg);
                pub.publish(msg);
                ros::spinOnce();
            } while(q != quadrant_I);
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff > accuracyConstAngle);
        } else {
            msg.angular.z = fast_rotate_cw;
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff < accuracyConstAngle);
        } 
    }

    /*Destination Angle in Quadrant II*/
    else if(dest_quadrant == quadrant_II && actual_quadrant == quadrant_I) {
        msg.angular.z = fast_rotate_ccw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff > accuracyConstAngle);
    } else if(dest_quadrant == quadrant_II && actual_quadrant == quadrant_III) {
        msg.angular.z = fast_rotate_cw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff < accuracyConstAngle);
    } else if(dest_quadrant == quadrant_II && actual_quadrant == quadrant_VI) {
        if((actual_angle - dest_angle) <= 180) {
            msg.angular.z = fast_rotate_cw;
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff < accuracyConstAngle);
        } else {
            msg.angular.z = fast_rotate_ccw;
            q = 0;
            do {
                q = getQuadrant(data.odometry_orientation_deg);
                pub.publish(msg);
                ros::spinOnce();
            } while(q != quadrant_I);
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff > accuracyConstAngle);
        } 
    } 

    /*Destination Angle in Quadrant III*/
    else if(dest_quadrant == quadrant_III && actual_quadrant == quadrant_VI) {
        msg.angular.z = fast_rotate_cw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff < accuracyConstAngle);
    } else if(dest_quadrant == quadrant_III && actual_quadrant == quadrant_II) {
        msg.angular.z = fast_rotate_ccw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff > accuracyConstAngle);
    } else if(dest_quadrant == quadrant_III && actual_quadrant == quadrant_I) {  
        if((actual_angle - dest_angle) <= 180) {
            msg.angular.z = fast_rotate_cw;
            q = 0;
            do {
                q = getQuadrant(data.odometry_orientation_deg);
                pub.publish(msg);
                ros::spinOnce();
            } while(q != quadrant_III);
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff > accuracyConstAngle);
        } else {
            msg.angular.z = fast_rotate_ccw;
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff < accuracyConstAngle);
        } 
    } 

    /*Destination Angle in Quadrant VI*/
    else if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_I) {
        msg.angular.z = fast_rotate_cw;
        q = 0;
        do {
            q = getQuadrant(data.odometry_orientation_deg);
            pub.publish(msg);
            ros::spinOnce();
        } while(q != quadrant_VI);
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff < accuracyConstAngle);
    } else if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_II) {
        if((dest_angle - actual_angle) <= 180) {
            msg.angular.z = fast_rotate_ccw;
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff > accuracyConstAngle);
        } else {
            msg.angular.z = fast_rotate_cw;
            q = 0;
            do {
                q = getQuadrant(data.odometry_orientation_deg);
                pub.publish(msg);
                ros::spinOnce();
            } while(q != quadrant_VI);
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff < accuracyConstAngle);
        }
    } else if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_III) {
        msg.angular.z = fast_rotate_ccw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff > accuracyConstAngle);
    }
}

double adjustOrientationForDestination(double dest_coords[]) {
    geometry_msgs::Twist msg;
    double deg_diff = 0;
    bool acc_deg = false;
    double position_offset_x = dest_coords[X_COORD] - data.odometry_positions[X_COORD];
    double position_offset_y = dest_coords[Y_COORD] - data.odometry_positions[Y_COORD];
    double dest_angle = rad2deg * calculateAngle(position_offset_x, position_offset_y);
    
    if (dest_angle < 0) {
        dest_angle = 180 + (180 + dest_angle);
    } else dest_angle = dest_angle;
    rotateToDestination(dest_angle, data.odometry_orientation_deg);
    msg.angular.z = 0;
    pub.publish(msg);
    ros::spinOnce();
    return dest_angle;
}

double angleCorrection(double dest_angle, double actual_angle) {
    double angle_offset;
    int dest_quadrant = getQuadrant(dest_angle);
    int actual_quadrant = getQuadrant(actual_angle);
    if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_I) {
        return -angular_correction;
    } else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_VI) {
        return angular_correction;
    } else if((dest_angle - actual_angle) < 0) {
        return -angular_correction;
    } else return angular_correction;
}

void moveToCoordinates(double *dest_coords) {
    geometry_msgs::Twist msg;
    double acc_x = 0; 
    double acc_y = 0;
    double acc_pos = 0;
    double dest_angle = adjustOrientationForDestination(dest_coords);
    msg.linear.x = 0.075;
    do {
        acc_x = std::fabs(dest_coords[X_COORD] - data.odometry_positions[X_COORD]);
        acc_y = std::fabs(dest_coords[Y_COORD] - data.odometry_positions[Y_COORD]);
        acc_pos = (acc_x + acc_y)/2;
        msg.angular.z = angleCorrection(dest_angle, data.odometry_orientation_deg);
        if(!scannedBeacon() && data.beacon_detected) {
            setLinearVelocity(0);
            beaconDetected(dest_coords);
            adjustOrientationForDestination(dest_coords);
            continue;
        }
        scannedBeacon(); 
        ros::spinOnce();
        pub.publish(msg);
        if (acc_pos < accuracyConstPosition) break;
    } while (true);
    ros::spinOnce();
    msg.linear.x = 0;
    msg.linear.z = 0;
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
        if(data.proximity_floor_values[idx] <= beacon_threshold) {
            data.beacon_detected = true;
            data.detecting_sensor_id = idx;
        } else {
            data.beacon_detected = false;
        }
        
        //ROS_INFO("SIDE LEFT: %zu, FRONT LEFT: %zu, FRONT LEFT: %zu, FRONT RIGHT: %zu",
        //                                data.proximity_floor_values[SIDE_LEFT],
        //                                data.proximity_floor_values[FRONT_LEFT],
        //                                data.proximity_floor_values[FRONT_RIGHT],
        //                                data.proximity_floor_values[SIDE_RIGHT]);
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
    if (y*rad2deg < 0) {
        data.odometry_orientation_deg = 180 + (180 + y*rad2deg);
    } else data.odometry_orientation_deg = y*rad2deg;
    ROS_INFO("x: %f, y: %f, theta: %f", data.odometry_positions[X_COORD],
                                        data.odometry_positions[Y_COORD], 
                                        data.odometry_orientation_deg);
                                        
}

void initYAML() {
    YAML::Node beacon;

    out << YAML::BeginSeq;

    out << YAML::EndSeq;
    std::ofstream fout(yaml_path);
    fout << out.c_str();
}

void appendBeaconArray() {
    int id = beacon_array[data.beacon_counter].id;
    beacon_array[data.beacon_counter].pose2d[X_COORD] = data.odometry_positions[X_COORD];
    beacon_array[data.beacon_counter].pose2d[Y_COORD] = data.odometry_positions[Y_COORD];
    beacon_array[data.beacon_counter].pose2d[THETA] = data.odometry_orientation_deg;
    ++data.beacon_counter;
}

void appendYAML(beacon &b) {
    //YAML::Node test = YAML::LoadFile("/homes/joschroeder/github/nightwatcher/catkin_ws/src/amiro_robot/amiro_gazebo/yaml/watchmen_route_0.yaml");
    out << YAML::BeginSeq;
    
    out << YAML::EndSeq;
    // TODO continue here
    std::ofstream fout(yaml_path);
    fout << out.c_str();
    //if(test["base"]["pose2d"][0].as<double>() == 2.5) ROS_INFO("TRUE");
    //double foo = test["base"]["pose2d"][0].as<double>();
    //ROS_INFO("foo: %f", test["base"]["pose2d"][0].as<double>());
}   


int getBeaconId() {
    if(data.proximity_floor_values[FRONT_LEFT] <= black_threshold &&
       data.proximity_floor_values[FRONT_RIGHT] <= black_threshold) {
        if(data.proximity_floor_values[SIDE_RIGHT] > low_threshold_gray_100 &&
           data.proximity_floor_values[SIDE_RIGHT] < high_threshold_gray_100) {
            if(data.proximity_floor_values[SIDE_LEFT] > low_threshold_gray_100 &&
                data.proximity_floor_values[SIDE_LEFT] < high_threshold_gray_100) {
                return 0;
            } else if(data.proximity_floor_values[SIDE_LEFT] > low_threshold_gray_125 &&
                        data.proximity_floor_values[SIDE_LEFT] < high_threshold_gray_125) {
                return 1;
            } else if(data.proximity_floor_values[SIDE_LEFT] > low_threshold_gray_150 &&
                        data.proximity_floor_values[SIDE_LEFT] < high_threshold_gray_150) {
                return 2; 
            }
        } else if(data.proximity_floor_values[SIDE_RIGHT] > low_threshold_gray_125 &&
                    data.proximity_floor_values[SIDE_RIGHT] < high_threshold_gray_125) {
            if(data.proximity_floor_values[SIDE_LEFT] > low_threshold_gray_100 &&
                data.proximity_floor_values[SIDE_LEFT] < high_threshold_gray_100) {
                return 3;
            } else if(data.proximity_floor_values[SIDE_LEFT] > low_threshold_gray_125 &&
                        data.proximity_floor_values[SIDE_LEFT] < high_threshold_gray_125) {
                return 4;
            } else if(data.proximity_floor_values[SIDE_LEFT] > low_threshold_gray_150 &&
                        data.proximity_floor_values[SIDE_LEFT] < high_threshold_gray_150) {
                return 5; 
            }
        } else if(data.proximity_floor_values[SIDE_RIGHT] > low_threshold_gray_150 &&
                    data.proximity_floor_values[SIDE_RIGHT] < high_threshold_gray_150) {
            if(data.proximity_floor_values[SIDE_LEFT] > low_threshold_gray_100 &&
                data.proximity_floor_values[SIDE_LEFT] < high_threshold_gray_100) {
                return 6;
            } else if(data.proximity_floor_values[SIDE_LEFT] > low_threshold_gray_125 &&
                        data.proximity_floor_values[SIDE_LEFT] < high_threshold_gray_125) {
                return 7;
            } else if(data.proximity_floor_values[SIDE_LEFT] > low_threshold_gray_150 &&
                        data.proximity_floor_values[SIDE_LEFT] < high_threshold_gray_150) {
                return 8; 
            }
        }
    }
}

bool scannedBeacon() {
    double pos_diff_x, pos_diff_y;
    for(int i = 0; i <= data.beacon_counter; ++i) {
        pos_diff_x = std::fabs(beacon_array[i].pose2d[X_COORD] - data.odometry_positions[X_COORD]);
        pos_diff_y = std::fabs(beacon_array[i].pose2d[Y_COORD] - data.odometry_positions[Y_COORD]);
        ROS_INFO("i = %i diff_x :%f diff_y %f", i, pos_diff_x, pos_diff_y);
        if((pos_diff_x < 0.2) && (pos_diff_y < 0.2)) {
            //ROS_INFO("TRUE");
            return true;
        } else {
            //ROS_INFO("FALSE");
            return false;
        }
    }

}

void scanBaseBeacon() {
    ros::spinOnce();
    base.id = getBeaconId();
    base.pose2d[X_COORD] = data.odometry_positions[X_COORD];
    base.pose2d[Y_COORD] = data.odometry_positions[Y_COORD];
    base.pose2d[THETA] = data.odometry_orientation_deg;
    beacon_array[0] = base;
    //++data.beacon_counter;
    ROS_INFO("%i", base.id);
    ROS_INFO("%f %f %f", base.pose2d[X_COORD], base.pose2d[Y_COORD], base.pose2d[THETA]);
    appendYAML(base);
}

bool proximityFrontLeftDetected() {
    if(data.proximity_floor_values[FRONT_LEFT] < beacon_threshold) return true; else return false;
}

bool proximityFrontRightDetected() {
    if(data.proximity_floor_values[FRONT_RIGHT] < beacon_threshold) return true; else return false;
}

bool proximitySideLeftDetected() {
    if(data.proximity_floor_values[SIDE_LEFT] < beacon_threshold) return true; else return false;
}

bool proximitySideRightDetected() {
    if(data.proximity_floor_values[SIDE_RIGHT] < beacon_threshold) return true; else return false;
}

bool proximityFrontBlack() {
    if(data.proximity_floor_values[FRONT_LEFT] <= black_threshold &&
       data.proximity_floor_values[FRONT_RIGHT] <= black_threshold) return true; else return false;
}

bool proximityAllDetected() {
    if(data.proximity_floor_values[FRONT_LEFT] < beacon_threshold &&
        data.proximity_floor_values[FRONT_RIGHT] < beacon_threshold &&
        data.proximity_floor_values[SIDE_LEFT] < beacon_threshold &&
        data.proximity_floor_values[SIDE_RIGHT] < beacon_threshold) return true; else return false;
}

void setAngularVelocity(double val) {
    geometry_msgs::Twist msg;
    msg.angular.z = val;
    pub.publish(msg);
    ros::spinOnce();
}

void setLinearVelocity(double val) {
    geometry_msgs::Twist msg;
    msg.linear.x = val;
    pub.publish(msg);
    ros::spinOnce();
}

void beaconDetected(double dest_coords[]) {
    //ROS_INFO("BEACON DETECTED");
    //if(scannedBeacon()) return;
    //ROS_INFO("NO RETURN");
    geometry_msgs::Twist msg;
    double orientation_diff, orientation_diff_half, orientation_new, deg_diff;
    int first_quadrant, second_quadrant, final_quadrant, q;
    ros::Time start;
    ros::Duration time_diff;
    ros::Duration ref_duration_1(1.00);
    ros::Duration ref_duration_2(3.00);
    double dur;
    setLinearVelocity(0);
    switch(data.detecting_sensor_id) {
        case FRONT_LEFT: {
            ROS_INFO("FRONT_LEFT");

            setLinearVelocity(0);
            msg.linear.x = 0.05;
            ROS_INFO("VEL 0.05");
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;
            ROS_INFO("VEL 0");
            
            msg.angular.z = slow_rotate_cw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontRightDetected());
            ROS_INFO("fronRight Done");
            msg.angular.z = 0;
            pub.publish(msg);
            data.odometry_orientation_beacon_first_contact = data.odometry_orientation_deg;
            ROS_INFO("first %f", data.odometry_orientation_beacon_first_contact);
            first_quadrant = getQuadrant(data.odometry_orientation_beacon_first_contact);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            ROS_INFO("FRONTLEFT DONE");
            msg.angular.z = 0;
            pub.publish(msg);
            data.odometry_orientation_beacon_second_contact = data.odometry_orientation_deg;
            second_quadrant = getQuadrant(data.odometry_orientation_beacon_second_contact);
            ROS_INFO("second %f", data.odometry_orientation_beacon_second_contact);
            if(first_quadrant == quadrant_VI && second_quadrant == quadrant_I) {
                orientation_diff = data.odometry_orientation_beacon_second_contact + 
                                    (360 - data.odometry_orientation_beacon_first_contact);
                ROS_INFO("IF: %f",  orientation_diff);
            } else {
                orientation_diff = (data.odometry_orientation_beacon_second_contact -
                                    data.odometry_orientation_beacon_first_contact);
                ROS_INFO("ELSE: %f", orientation_diff);
            }
            orientation_diff_half = orientation_diff / 2;
            ROS_INFO("diff_half: %f", orientation_diff_half);
            orientation_new = data.odometry_orientation_beacon_second_contact - 
                                orientation_diff_half;
            if(orientation_new < 0) { 
                orientation_new = 360 + orientation_new;
            }
            final_quadrant = getQuadrant(orientation_new);
            ROS_INFO("orientation_new: %f", orientation_new);
            msg.angular.z = slow_rotate_cw;
            pub.publish(msg);
            if(second_quadrant == quadrant_I && final_quadrant == quadrant_VI) {
                if((orientation_new < 0.5) || ((360 - orientation_new) < 0.5)) {
                ROS_INFO("IFIF");
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(q != quadrant_VI);    
                } else {
                    ROS_INFO("IFELSE");
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(q != quadrant_VI);
                    do {
                        deg_diff = data.odometry_orientation_deg - orientation_new;
                        //ROS_INFO("deg_diff_IF: %f", deg_diff);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(deg_diff > accuracyConstAngle);
                }
            } else {
                ROS_INFO("ELSE");
                if((orientation_new < 0.5)) {
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(q != quadrant_VI);
                } else {
                    do {
                        deg_diff = data.odometry_orientation_deg - orientation_new; 
                        //ROS_INFO("deg_diff: %f", deg_diff);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(deg_diff < accuracyConstAngle);
                }
            }
            msg.angular.z = 0;
            pub.publish(msg);
            msg.linear.x = -0.02;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected() && proximityFrontLeftDetected());
            msg.linear.x = 0;
            pub.publish(msg);
            msg.linear.x = 0.035;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while(time_diff < ref_duration_2);
            msg.linear.x = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(!proximityFrontBlack());
            msg.linear.x = 0;
            msg.angular.z = 0;
            pub.publish(msg);
            appendBeaconArray();
            //getBeaconId();
            //moveToCoordinates(dest_coords);
            break;
            /*
            //YAML
            break;*/
        }
        case FRONT_RIGHT: {
            msg.angular.z = slow_rotate_cw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            setAngularVelocity(0);
            data.odometry_orientation_beacon_first_contact = data.odometry_orientation_deg;
            first_quadrant = getQuadrant(data.odometry_orientation_beacon_first_contact);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontRightDetected());
            setAngularVelocity(0);
            data.odometry_orientation_beacon_second_contact = data.odometry_orientation_deg;
            second_quadrant = getQuadrant(data.odometry_orientation_beacon_second_contact);
            if(first_quadrant == quadrant_I && second_quadrant == quadrant_VI) {
                orientation_diff = data.odometry_orientation_beacon_first_contact -
                                    (360 - data.odometry_orientation_beacon_second_contact);
            } else {
                orientation_diff = data.odometry_orientation_beacon_first_contact -
                                    data.odometry_orientation_beacon_second_contact;
            }
            orientation_diff_half = orientation_diff / 2;
            orientation_new = data.odometry_orientation_beacon_first_contact -
                                orientation_diff_half;
            if(orientation_new < 0) {
                orientation_new = 360 + orientation_new;
            }
            final_quadrant = getQuadrant(data.odometry_orientation_deg);
            if(second_quadrant == quadrant_VI && final_quadrant == quadrant_I) {
                msg.angular.z = slow_rotate_ccw;
                do {
                    q = getQuadrant(data.odometry_orientation_deg);
                    pub.publish(msg);
                    ros::spinOnce();
                } while(q != quadrant_I);
                do {
                    deg_diff = orientation_new - data.odometry_orientation_deg;
                    pub.publish(msg);
                    ros::spinOnce();
                } while(deg_diff > accuracyConstAngle);
            } else {
                msg.angular.z = slow_rotate_ccw;
                do {
                    deg_diff = orientation_new - data.odometry_orientation_deg;
                    pub.publish(msg);
                    ros::spinOnce();
                } while(deg_diff > accuracyConstAngle);
            }
            msg.angular.z = 0;
            msg.linear.x = -0.002;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected() && proximityFrontRightDetected());
            setLinearVelocity(0);
            msg.linear.x = 0.065;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while(time_diff < ref_duration_2);
            setLinearVelocity(0);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(!proximityFrontBlack());
            setAngularVelocity(0);
            getBeaconId();
            //ID; POSE THETAAAAAAAAAAAAAAAAAAA
            moveToCoordinates(dest_coords);
            break;
        }
        case SIDE_LEFT: {
            ROS_INFO("SIDE_LEFT");
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(!proximityFrontLeftDetected());
            setAngularVelocity(0);
            data.odometry_orientation_beacon_first_contact = data.odometry_orientation_deg;
            first_quadrant = getQuadrant(data.odometry_orientation_beacon_first_contact);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            setAngularVelocity(0);
            data.odometry_orientation_beacon_second_contact = data.odometry_orientation_deg;
            second_quadrant = getQuadrant(data.odometry_orientation_beacon_second_contact);
            if(first_quadrant == quadrant_VI && second_quadrant == quadrant_I) {
                orientation_diff = data.odometry_orientation_beacon_second_contact -
                                    (360 - data.odometry_orientation_beacon_first_contact);
            } else {
                orientation_diff = (data.odometry_orientation_beacon_second_contact
                                - data.odometry_orientation_beacon_first_contact);
            }
            orientation_diff_half = orientation_diff / 2;
            orientation_new = data.odometry_orientation_beacon_second_contact - 
                                orientation_diff_half;
            if(orientation_new < 0) {
                orientation_new = 360 + orientation_new;
            }
            final_quadrant = getQuadrant(orientation_new);
            if(second_quadrant == quadrant_I && final_quadrant == quadrant_VI) {
                msg.angular.z = slow_rotate_cw;
                do {
                    q = getQuadrant(data.odometry_orientation_deg);
                    pub.publish(msg);
                    ros::spinOnce();
                } while(q != quadrant_VI);
                do {
                    deg_diff = data.odometry_orientation_deg - orientation_new;
                } while(deg_diff < accuracyConstAngle);
            } else {
                msg.angular.z = slow_rotate_cw;
                do {
                    deg_diff = data.odometry_orientation_deg - orientation_new;
                    pub.publish(msg);
                    ros::spinOnce();
                } while(deg_diff > accuracyConstAngle);
            }
            msg.angular.z = 0;
            msg.linear.x = -0.002;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected() && proximityFrontRightDetected());
            setLinearVelocity(0);
            msg.linear.x = 0.065;
            ros::Time start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start; 
            } while(time_diff < ref_duration_2);
            msg.linear.x = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(!proximityFrontBlack());
            msg.angular.z = 0;
            pub.publish(msg);
            getBeaconId();
            //ID, pose2D und theta in YAML-Datei speichern
            moveToCoordinates(dest_coords);
            break;
        }
        case SIDE_RIGHT: {
            ROS_INFO("SIDE_RIGHT");
            msg.angular.z = slow_rotate_cw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(!proximityFrontRightDetected());
            setAngularVelocity(0);
            data.odometry_orientation_beacon_first_contact = data.odometry_orientation_deg;
            first_quadrant = getQuadrant(data.odometry_orientation_beacon_first_contact);
            msg.angular.z = slow_rotate_cw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontRightDetected());
            setAngularVelocity(0);
            data.odometry_orientation_beacon_second_contact = data.odometry_orientation_deg;
            second_quadrant = getQuadrant(data.odometry_orientation_beacon_second_contact);
            if(first_quadrant == quadrant_I && second_quadrant == quadrant_VI) {
                orientation_diff = data.odometry_orientation_beacon_first_contact -
                                    (360 - data.odometry_orientation_beacon_second_contact);
            } else {
                orientation_diff = data.odometry_orientation_beacon_first_contact - 
                                    data.odometry_orientation_beacon_second_contact;
            } 
            orientation_diff_half = orientation_diff / 2;
            orientation_new = data.odometry_orientation_beacon_first_contact - 
                                orientation_diff_half;
            if(orientation_new < 0) {
                orientation_new = 360 + orientation_new;
            }
            final_quadrant = getQuadrant(data.odometry_orientation_deg);
            if(second_quadrant == quadrant_VI && final_quadrant == quadrant_I) {
                msg.angular.z = slow_rotate_ccw;
                do {
                    q = getQuadrant(data.odometry_orientation_deg);
                    pub.publish(msg);
                    ros::spinOnce();
                } while(q != quadrant_I);
                do {
                    deg_diff = orientation_new - data.odometry_orientation_deg;
                    pub.publish(msg);
                    ros::spinOnce();
                } while(deg_diff > accuracyConstAngle);
            } else {
                msg.angular.z = slow_rotate_ccw;
                do {
                    deg_diff = orientation_new - data.odometry_orientation_deg;
                    pub.publish(msg);
                    ros::spinOnce();
                } while(deg_diff > accuracyConstAngle);
            }
            msg.angular.z = 0;
            msg.linear.x = -0.002;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected() && proximityFrontRightDetected());
            setLinearVelocity(0);
            msg.linear.x = 0.065;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start; 
            } while(time_diff < ref_duration_2);
            setLinearVelocity(0);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(!proximityFrontBlack());
            setAngularVelocity(0);
            getBeaconId();
            //ID, position & theta in YAML-Datei speichern
            moveToCoordinates(dest_coords);
            break;
        }
    }
}

void moveToMiddleOfTheMap() {
    double middle_of_map[2] = {X_COORD_MID_MAP, Y_COORD_MID_MAP};
    moveToCoordinates(middle_of_map);
}

void exploration() {
    double last_hop[2];
    int even_sign = 1, uneven_sign = 1, uneven_step_counter = 1, even_step_counter = 1;
    bool even_step = false;
    double next_hop_x = 0, next_hop_y = 0, last_hop_x = 0, last_hop_y = 0;
    double next_hop_coords[2]{0, 0};
    last_hop_x = data.odometry_positions[X_COORD];
    last_hop_y = data.odometry_positions[Y_COORD];

    moveToMiddleOfTheMap();
    int i = 1;
    do {  
        if(!even_step) {    
            next_hop_x = last_hop_x + (even_step_counter * amiro_base_diameter * uneven_sign);
            next_hop_y = last_hop_y;
            even_step = true;
            uneven_step_counter++;
            uneven_sign *= (-1);
            next_hop_coords[X_COORD] = next_hop_x;
            next_hop_coords[Y_COORD] = next_hop_y;
            ROS_INFO("%i. HOP: %f, %f", i, next_hop_x, next_hop_y);
            moveToCoordinates(next_hop_coords);
            last_hop_x = next_hop_x;
            last_hop_y = next_hop_y;
            i++;
        } else {
            next_hop_x = last_hop_x;
            next_hop_y = last_hop_y + (even_step_counter * amiro_base_diameter * even_sign);
            even_step = false;
            even_step_counter++;
            even_sign *= (-1);
            next_hop_coords[X_COORD] = next_hop_x;
            next_hop_coords[Y_COORD] = next_hop_y;
            ROS_INFO("%i. HOP: %f, %f", i, next_hop_x, next_hop_y);
            moveToCoordinates(next_hop_coords);
            last_hop_x = next_hop_x;
            last_hop_y = next_hop_y;
            i++;
        }
        
    } while(i <= 14);
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

    double dest_coords[2] = {4, 2.5};
    ros::Duration(3.0).sleep();
    scanBaseBeacon();
    scannedBeacon();
    moveToCoordinates(dest_coords);
    //adjustOrientationForDestination(dest_coords);
    //exploration();
    //ROS_INFO("1. HOP DONE");
    //dest_coords[0] = 2;
    //dest_coords[1] = 2;
    //moveToCoordinates(dest_coords);
    //ROS_INFO("2. HOP DONE");
    //ros::Duration(1.0).sleep();
    //dest_coords[0] = 2.4;
    //dest_coords[1] = 2.6;
    //moveToCoordinates(dest_coords);
    //dest_coords[0] = 2.4;
    //dest_coords[1] = 2.4;
    //moveToCoordinates(dest_coords);
    /*while(ros::ok()) {
        msg.angular.z = 0.35;
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }*/
        return 0;
}
