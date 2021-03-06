#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <amiro_msgs/UInt16MultiArrayStamped.h>
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"
#include <tf/tf.h>
#include <math.h>
#include "sensor_msgs/Image.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "yaml-cpp/yaml.h"
#include <fstream>
#define BEACON_FOUND 1
#define NO_BEACON_FOUND 0
#define FRONT_RIGHT 0
#define FRONT_LEFT 3
#define SIDE_RIGHT 1
#define SIDE_LEFT 2
#define X_COORD 0
#define Y_COORD 1
#define THETA 2
#define ID 3
#define BASE 0 
#define BEACON_0 1
#define BEACON_1 2
#define BEACON_2 3
#define BEACON_3 4
#define BEACON_4 5
#define BEACON_5 6
#define X_COORD_MID_MAP 2.50
#define Y_COORD_MID_MAP 2.50
#define accuracyConstPosition 0.0075
#define accuracyConstAngle 0.001
#define rad2deg 180/M_PI
#define deg2rad M_PI/180
#define fast_rotate_ccw 0.45
#define fast_rotate_cw -0.45
#define slow_rotate_ccw 0.3
#define slow_rotate_cw -0.3
#define super_slow_rotate_ccw 0.15
#define super_slow_rotate_cw -0.15
#define mega_slow_rotate_ccw 0.075
#define mega_slow_rotate_cw -0.075
#define angular_offset 1e-6
#define angular_correction 0.025
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
#define quadrant_I 1
#define quadrant_II 2
#define quadrant_III 3
#define quadrant_VI 4
#define angle_error_per_sec 0.002
#define angular_error_after_failed_rescue 38

/*ros::Publisher*/
ros::Publisher pub;

/*YAML::Emitter*/
YAML::Emitter out;

// NOTE Zielpfad der .yaml-Datei, Bitte vor dem Testen ändern!
std::string yaml_path = ""; 

struct beacon {
    int id;
    double pose2d[3];
    std::string name;
};

beacon beacon_array[24];

/*Beacon vectors*/
double scanned_beacons[7][4];
double distance_array[7];
std::vector<std::vector<double> > beacon_vector;
std::vector<double> base;
std::vector<double> beacon_0;
std::vector<double> beacon_1;
std::vector<double> beacon_2;
std::vector<double> beacon_3;
std::vector<double> beacon_4;
std::vector<double> beacon_5;



/*Global Struct*/
struct roboData {
    bool beacon_detected;
    bool near_beacon;
    int detecting_sensor_id;
    int beacon_counter = 0;
    int correction_degree = 0;
    double angle_correction_encoder = 0;
    double proximity_floor_values[4];
    double init_positions[2] = {0, 0};
    double odometry_positions[2];
    double odometry_positions_encoder[2];
    double odometry_orientation_deg;
    double odometry_orientation_deg_encoder;
    double odometry_orientation_beacon_first_contact;
    double odometry_orientation_beacon_second_contact;
    double encoder_position_error_x = 0;
    double encoder_position_error_y = 0;
    double angular_error_per_degree = 0.096;
    double offset_after_calibration = 0;
    int shortest_path_order[7];
    int order_counter = 1;
    int hop = 0;
} data;

/*Prototypes*/
int getQuadrant(double angle);
int getBeaconId();
bool scannedBeacon();
bool distanceToBeacon();
bool proximityFrontLeftDetected();
bool proximityFrontRightDetected();
bool proximitySideLeftDetected();
bool proximitySideRightDetected();
bool proximityFrontBlack();
bool proximityFrontLeftBlack();
bool proximityFrontRightBlack();
bool proximityAllDetected();
bool beaconBelow();
void resetBeaconDetection();
void appendBeaconArray();
void printBeaconArray();
void appendYAML();
void setAngularVelocity(double val);
void setLinearVelocity(double val);
void getInitPosition();
void moveToCoordinates(double *dest_coords);
void moveToCoordinatesEncoder(double *dest_coords);
void rotateToDestination(double dest_angle, double actual_angle);
void floorProximityCallback(const amiro_msgs::UInt16MultiArrayStamped &msg);
void odometryDataCallback(const nav_msgs::Odometry &msg);
void beaconDetection();
void exploration();
double rotateToDestinationEncoder(double dest_angle, double actual_angle);
double adjustOrientationForDestination(double dest_coords[]);
double adjustOrientationForDestinationEncoder(double dest_coords[]);
double angleCorrection(double dest_angle, double actual_angle);

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

bool quadrantMatch(int dest_quadrant, int actual_quadrant) {
    if(dest_quadrant == actual_quadrant) {
        return true;
    }  else {
        return false;
    } 
}

void rotateToDestination(double dest_angle, double actual_angle) {
    int dest_quadrant = getQuadrant(dest_angle);
    int actual_quadrant = getQuadrant(actual_angle);
    ROS_INFO("dest_quadrant: %i", dest_quadrant);
    ROS_INFO("act_quadrant: %i", actual_quadrant);
    ROS_INFO("dest_angle: %f", dest_angle);
    ROS_INFO("actual_angle: %f", actual_angle);
    double deg_diff = 0;
    int q = 0;
    geometry_msgs::Twist msg;

    /*Destination Angle and Actual Angle in same Quadrant*/
    if(dest_quadrant == actual_quadrant) {
        if(dest_angle > actual_angle) {
            msg.angular.z = fast_rotate_ccw; 
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
                q = getQuadrant(data.odometry_orientation_deg);
                if(!quadrantMatch(dest_quadrant, q)) break;
            } while((deg_diff > accuracyConstAngle));
            msg.angular.z = 0;
            pub.publish(msg);
        } else {
            msg.angular.z = fast_rotate_cw;
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg;
                q = getQuadrant(data.odometry_orientation_deg);
                pub.publish(msg);
                ros::spinOnce();
                if(!quadrantMatch(dest_quadrant, q)) break;
            } while((deg_diff < accuracyConstAngle));
            msg.angular.z = 0;
            pub.publish(msg); 
        }
    } 

    /*Destination Angle in Quadrant I*/
    else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_II) {
        msg.angular.z = fast_rotate_cw;
        do {
            q = getQuadrant(data.odometry_orientation_deg);
            deg_diff = dest_angle - data.odometry_orientation_deg;
            ROS_INFO("deg_diff: %f", deg_diff);
            pub.publish(msg);
            ros::spinOnce();
            if(q == quadrant_VI) break;
        } while(deg_diff < accuracyConstAngle);
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
                q = getQuadrant(data.odometry_orientation_deg);
                if(dest_angle < 0.5) {
                    if (q == quadrant_I) break;
                }
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff > accuracyConstAngle);
        } else {
            msg.angular.z = fast_rotate_cw;
            do {
                q = getQuadrant(data.odometry_orientation_deg);
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
                if(q == quadrant_VI) break;
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
                if((360 - dest_angle) < 0.5) break; 
                deg_diff = dest_angle - data.odometry_orientation_deg;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff < accuracyConstAngle);
        }
    } else if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_III) {
        msg.angular.z = fast_rotate_ccw;
        do {
            q = getQuadrant(data.odometry_orientation_deg);
            deg_diff = dest_angle - data.odometry_orientation_deg;
            pub.publish(msg);
            ros::spinOnce();
            if(q == quadrant_I) break;
        } while(deg_diff > accuracyConstAngle);
    }
}

int getCorrectionDirection(double dest_angle, double actual_angle) {
    // ccw = +1, cw = -1;
    int dest_quadrant = getQuadrant(dest_angle);
    int actual_quadrant = getQuadrant(actual_angle);

    /*dest_quadrant == actual_quadrant*/
    if(dest_quadrant == actual_quadrant) {
        if(dest_angle > actual_angle) return 1; else return -1;
    }

    /*dest_quadrant == I*/
    else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_VI) return 1;
    else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_III) {
        if ((actual_angle - dest_angle) <= 180) return 1; else return -1;
    } else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_II) return -1; 
        
    /*dest_quadrant == II*/
    else if(dest_quadrant == quadrant_II && actual_quadrant == quadrant_VI) {
        if((actual_angle - dest_angle) <= 180) return -1; else return 1;
    } else if(dest_quadrant == quadrant_II && actual_quadrant == quadrant_III) return -1;
    else if(dest_quadrant == quadrant_II && actual_quadrant == quadrant_I) return 1;

    /*dest_quadrant == III*/
    else if(dest_quadrant == quadrant_III && actual_quadrant == quadrant_VI) return -1;
    else if(dest_quadrant == quadrant_III && actual_quadrant == quadrant_II) return 1;
    else if(dest_quadrant == quadrant_III && actual_quadrant == quadrant_I) {
        if((actual_angle - dest_angle) <= 180) return -1; else return 1;
    }

    /*dest_quadrant == VI*/
    else if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_III) return 1;
    else if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_II) {
        if((dest_angle - actual_angle) <= 180) return 1; else return -1;
    } else if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_I) return -1;
}

double rotateToDestinationEncoder(double dest_angle, double actual_angle) {
    int dest_quadrant = getQuadrant(dest_angle);
    int actual_quadrant = getQuadrant(actual_angle);
    int correction_direction = getCorrectionDirection(dest_angle, actual_angle);
    //ROS_INFO("dest_quadrant: %i", dest_quadrant);
    //ROS_INFO("act_quadrant: %i", actual_quadrant);
    //ROS_INFO("dest_angle: %f", dest_angle);
    //ROS_INFO("actual_angle: %f", actual_angle);
    double deg_diff = 0;
    int q = 0;
    geometry_msgs::Twist msg;
    double correction = (dest_angle * data.angular_error_per_degree) /*+ data.offset_after_calibration*/;
    ROS_INFO("dest_angle_old: %f", dest_angle);
    //dest_angle = dest_angle + (correction * correction_direction * data.angular_error_per_degree); 
    dest_angle = dest_angle + (correction_direction * data.offset_after_calibration);
    ROS_INFO("dest_angle : %f", dest_angle);
    if(dest_angle > 360) {
        dest_angle = dest_angle - 360;
    } else if(dest_angle < 0) {
        dest_angle  = dest_angle + 360;
    }
    ROS_INFO("correction: %f", correction);
    ROS_INFO("dest_angle_new: %f", dest_angle);
    ros::Time start;
    ros::Duration time_diff;
    ros::Duration ref_duration_1(1.00);
    
    /*Destination Angle and Actual Angle in same Quadrant*/
    if(dest_quadrant == actual_quadrant) {
        if(dest_angle > actual_angle) {
            //correction = dest_angle - actual_angle;
            //ROS_INFO("correction: %f", correction);
            msg.angular.z = slow_rotate_ccw; 
            do {
                deg_diff = dest_angle - (data.odometry_orientation_deg_encoder);
                //ROS_INFO("deg_diff: %f", deg_diff);
                pub.publish(msg);
                ros::spinOnce();
                q = getQuadrant(data.odometry_orientation_deg_encoder);
                if(!quadrantMatch(dest_quadrant, q)) break;
            } while((deg_diff > accuracyConstAngle));
        } else {
            //correction = actual_angle - dest_angle;
            //ROS_INFO("correction: %f", correction);
            msg.angular.z = slow_rotate_cw;
            do {
                deg_diff = dest_angle - (data.odometry_orientation_deg_encoder);
                q = getQuadrant(data.odometry_orientation_deg_encoder);
                pub.publish(msg);
                ros::spinOnce();
                if(!quadrantMatch(dest_quadrant, q)) break;
            } while((deg_diff < accuracyConstAngle));
        }
    } 

    /*Destination Angle in Quadrant I*/
    else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_II) {
        msg.angular.z = slow_rotate_cw;
        do {
            q = getQuadrant(data.odometry_orientation_deg_encoder);
            deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
            pub.publish(msg);
            ros::spinOnce();
            if(q == quadrant_VI) break;
        } while(deg_diff < accuracyConstAngle);
    } else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_VI) {
        msg.angular.z = slow_rotate_ccw;
        q = 0;
        do {
            q = getQuadrant(data.odometry_orientation_deg_encoder);
            pub.publish(msg);
            ros::spinOnce(); 
        } while(q != quadrant_I);
        //correction = dest_angle - actual_angle;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff > accuracyConstAngle);
        msg.angular.z = 0;
        pub.publish(msg);
    } else if(dest_quadrant == quadrant_I && actual_quadrant == quadrant_III) {
        if((actual_angle - dest_angle) <= 180) {
            msg.angular.z = slow_rotate_ccw; 
            q = 0;
            do {
                q = getQuadrant(data.odometry_orientation_deg_encoder);
                pub.publish(msg);
                ros::spinOnce();
            } while(q != quadrant_I);
            do {
                q = getQuadrant(data.odometry_orientation_deg_encoder);
                deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
                pub.publish(msg);
                ros::spinOnce();
                if(q == quadrant_VI) break;
            } while(deg_diff > accuracyConstAngle);
            msg.angular.z = 0;
            pub.publish(msg);
        } else {
            msg.angular.z = slow_rotate_cw;
            do {
                q = getQuadrant(data.odometry_orientation_deg_encoder);
                deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
                pub.publish(msg);
                ros::spinOnce();
                if(q == quadrant_VI) break;
            } while(deg_diff < accuracyConstAngle);
            msg.angular.z = 0;
            pub.publish(msg);

        } 
    }

    /*Destination Angle in Quadrant II*/
    else if(dest_quadrant == quadrant_II && actual_quadrant == quadrant_I) {
        msg.angular.z = slow_rotate_ccw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff > accuracyConstAngle);
    } else if(dest_quadrant == quadrant_II && actual_quadrant == quadrant_III) {
        msg.angular.z = slow_rotate_cw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff < accuracyConstAngle);
    } else if(dest_quadrant == quadrant_II && actual_quadrant == quadrant_VI) { 
        if((actual_angle - dest_angle) <= 180) {
            msg.angular.z = slow_rotate_cw;
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff < accuracyConstAngle);
        } else {
            msg.angular.z = slow_rotate_ccw;
            q = 0;
            do {
                q = getQuadrant(data.odometry_orientation_deg_encoder);
                pub.publish(msg);
                ros::spinOnce();
            } while(q != quadrant_I);
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff > accuracyConstAngle);
        } 
    } 

    /*Destination Angle in Quadrant III*/
    else if(dest_quadrant == quadrant_III && actual_quadrant == quadrant_VI) {
        msg.angular.z = slow_rotate_cw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff < accuracyConstAngle);
    } else if(dest_quadrant == quadrant_III && actual_quadrant == quadrant_II) {
        msg.angular.z = slow_rotate_ccw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff > accuracyConstAngle);
    } else if(dest_quadrant == quadrant_III && actual_quadrant == quadrant_I) {  
        if((actual_angle - dest_angle) <= 180) { 
            msg.angular.z = slow_rotate_cw;
            q = 0;
            do {
                q = getQuadrant(data.odometry_orientation_deg_encoder);
                pub.publish(msg);
                ros::spinOnce();
            } while(q != quadrant_III);
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff > accuracyConstAngle);
        } else {
            msg.angular.z = slow_rotate_ccw;
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff < accuracyConstAngle);
        } 
    } 

    /*Destination Angle in Quadrant VI*/
    else if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_I) {
        msg.angular.z = slow_rotate_cw;
        q = 0;
        do {
            q = getQuadrant(data.odometry_orientation_deg_encoder);
            pub.publish(msg);
            ros::spinOnce();
        } while(q != quadrant_VI);
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff < accuracyConstAngle);
    } else if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_II) {
        if((dest_angle - actual_angle) <= 180) {
            msg.angular.z = slow_rotate_ccw;
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff > accuracyConstAngle);
        } else {
            msg.angular.z = slow_rotate_cw;
            q = 0;
            do {
                q = getQuadrant(data.odometry_orientation_deg_encoder);
                pub.publish(msg);
                ros::spinOnce();
            } while(q != quadrant_VI);
            do {
                deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
                pub.publish(msg);
                ros::spinOnce();
            } while(deg_diff < accuracyConstAngle);
        }
    } else if(dest_quadrant == quadrant_VI && actual_quadrant == quadrant_III) {
        msg.angular.z = slow_rotate_ccw;
        do {
            deg_diff = dest_angle - data.odometry_orientation_deg_encoder;
            pub.publish(msg);
            ros::spinOnce();
        } while(deg_diff > accuracyConstAngle);
    }
    return dest_angle;
}

void beaconDetectionTimeBased() {
    geometry_msgs::Twist msg;
    ros::Time start, end;
    ros::Duration time_diff, first_contact, second_contact;
    double half_arc_duration, full_arc_duration, arc_duration;
    ros::Duration ref_duration_1(1.00);
    ros::Duration ref_duration_2(3.00);
    msg.linear.x = 0;
    pub.publish(msg);
    ROS_INFO("BEACON DETECTED: %i", data.detecting_sensor_id);
    switch (data.detecting_sensor_id) {
        case FRONT_LEFT: {
            msg.linear.x = 0.03;
            pub.publish(msg);
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_cw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontRightDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_ccw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            full_arc_duration = (ros::Time::now().toSec() - start.toSec());
            msg.angular.z = 0;
            pub.publish(msg);
            half_arc_duration = full_arc_duration / 2 ;
            //ROS_INFO("half: %f", half_arc_duration);
            msg.angular.z = slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                arc_duration = ros::Time::now().toSec() - start.toSec();
            } while(arc_duration < half_arc_duration);
            msg.angular.z = 0;
            pub.publish(msg);
            msg.linear.x = -0.02;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected() && proximityFrontRightDetected());
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
            if(proximityFrontRightBlack() || proximityFrontLeftBlack()) {
                msg.angular.z = slow_rotate_ccw;
                start = ros::Time::now();
                do {
                    pub.publish(msg);
                    ros::spinOnce();
                    time_diff = ros::Time::now() - start;
                } while(time_diff < ref_duration_2);
            } 
            msg.angular.z = super_slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(!proximityFrontLeftBlack());
            msg.angular.z = 0;
            pub.publish(msg);
            msg.angular.z = mega_slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftBlack());
            full_arc_duration = ros::Time::now().toSec() - start.toSec();
            half_arc_duration = full_arc_duration / 2;
            msg.angular.z = 0;
            pub.publish(msg);
            msg.angular.z = mega_slow_rotate_ccw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                arc_duration = ros::Time::now().toSec() - start.toSec();
            }while(arc_duration < (half_arc_duration - 0.5));
            msg.angular.z = 0;
            pub.publish(msg);
            break;
        }
        case FRONT_RIGHT: {
            msg.linear.x = 0.03;
            pub.publish(msg);
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontRightDetected());
            full_arc_duration = ros::Time::now().toSec() - start.toSec();
            msg.angular.z = 0;
            pub.publish(msg);
            half_arc_duration = full_arc_duration / 2;
            msg.angular.z = slow_rotate_ccw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                arc_duration = ros::Time::now().toSec() - start.toSec();
            } while(arc_duration < half_arc_duration);
            msg.angular.z = 0;
            pub.publish(msg);
            msg.linear.x = -0.02;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected() && proximityFrontRightDetected());
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
            if(proximityFrontRightBlack() || proximityFrontLeftBlack()) {
                msg.angular.z = slow_rotate_ccw;
                start = ros::Time::now();
                do {
                    pub.publish(msg);
                    ros::spinOnce();
                    time_diff = ros::Time::now() - start;
                } while(time_diff < ref_duration_2);
            } 
            msg.angular.z = super_slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(!proximityFrontLeftBlack());
            msg.angular.z = 0;
            pub.publish(msg);
            msg.angular.z = mega_slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftBlack());
            full_arc_duration = ros::Time::now().toSec() - start.toSec();
            half_arc_duration = full_arc_duration / 2;
            msg.angular. z = 0;
            pub.publish(msg);
            msg.angular.z = mega_slow_rotate_ccw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                arc_duration = ros::Time::now().toSec() - start.toSec();
            } while(arc_duration < (half_arc_duration - 0.5));
            msg.angular.z = 0;
            pub.publish(msg);
            break;
        }
        case SIDE_LEFT: {
            msg.linear.x = 0.002;
            pub.publish(msg);
            msg.angular.z = 0.8;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
                if(proximityFrontLeftDetected() && proximityFrontRightDetected()) break;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontRightDetected());
            full_arc_duration = ros::Time::now().toSec() - start.toSec();
            msg.angular.z = 0;
            pub.publish(msg);
            half_arc_duration = full_arc_duration / 2;
            msg.angular.z = slow_rotate_ccw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                arc_duration = ros::Time::now().toSec() - start.toSec();
            } while(arc_duration < half_arc_duration);
            msg.angular.z = 0;
            pub.publish(msg);
            msg.linear.x = -0.02;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected() && proximityFrontRightDetected());
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
            if(proximityFrontRightBlack() || proximityFrontLeftBlack()) {
                msg.angular.z = slow_rotate_ccw;
                start = ros::Time::now();
                do {
                    pub.publish(msg);
                    ros::spinOnce();
                    time_diff = ros::Time::now() - start;
                } while(time_diff < ref_duration_2);
            } 
            msg.angular.z = super_slow_rotate_cw; //AJSFHJASHFJ
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(!proximityFrontLeftBlack());
            msg.angular.z = 0;
            pub.publish(msg);
            msg.angular.z = mega_slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftBlack());
            full_arc_duration = ros::Time::now().toSec() - start.toSec();
            half_arc_duration = full_arc_duration / 2;
            msg.angular. z = 0;
            pub.publish(msg);
            msg.angular.z = mega_slow_rotate_ccw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                arc_duration = ros::Time::now().toSec() - start.toSec();
            } while((arc_duration) < (half_arc_duration - 0.5));
            msg.angular.z = 0;
            pub.publish(msg);
            msg.linear.x = 0.002;
            pub.publish(msg);
            msg.angular.z = -0.8;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
                //if(proximityFrontLeftDetected() && proximityFrontRightDetected()) break;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;
            pub.publish(msg);
            break;
        }
        case SIDE_RIGHT: {
            msg.linear.x = 0.002;
            pub.publish(msg);
            msg.angular.z = -0.8;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
                if(proximityFrontLeftDetected() && proximityFrontRightDetected()) break;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_cw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontRightDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_ccw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            full_arc_duration = ros::Time::now().toSec() - start.toSec();
            msg.angular.z = 0;
            pub.publish(msg);
            half_arc_duration = full_arc_duration / 2;
            msg.angular.z = slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                arc_duration = ros::Time::now().toSec() - start.toSec();
            } while(arc_duration < half_arc_duration);
            msg.angular.z = 0;
            pub.publish(msg);
            msg.linear.x = -0.02;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected() && proximityFrontRightDetected());
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
            if(proximityFrontRightBlack() || proximityFrontLeftBlack()) {
                msg.angular.z = slow_rotate_ccw;
                start = ros::Time::now();
                do {
                    pub.publish(msg);
                    ros::spinOnce();
                    time_diff = ros::Time::now() - start;
                } while(time_diff < ref_duration_2);
            } 
            msg.angular.z = super_slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(!proximityFrontLeftBlack());
            msg.angular.z = 0;
            pub.publish(msg);
            msg.angular.z = mega_slow_rotate_cw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftBlack());
            full_arc_duration = ros::Time::now().toSec() - start.toSec();
            half_arc_duration = full_arc_duration / 2;
            msg.angular. z = 0;
            pub.publish(msg);
            msg.angular.z = mega_slow_rotate_ccw;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                arc_duration = ros::Time::now().toSec() - start.toSec();
            } while(arc_duration < (half_arc_duration - 0.5));
            msg.angular.z = 0;
            pub.publish(msg);
            msg.linear.x = 0.002;
            pub.publish(msg);
            msg.angular.z = 0.8;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
                //if(proximityFrontLeftDetected() && proximityFrontRightDetected()) break;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;
            pub.publish(msg);
            break;
        }
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
    ros::Duration time_diff;
    ros::Duration safety_duration(1.00);
    ros::Time start;
    double acc_x = 0; 
    double acc_y = 0;
    double acc_pos = 0;
    double acc_pos_new = 0;
    double acc_pos_min;
    double areal_threshold = 0.012;
    double dest_angle = adjustOrientationForDestination(dest_coords);
    acc_x = std::fabs(dest_coords[X_COORD] - data.odometry_positions[X_COORD]);
    acc_y = std::fabs(dest_coords[Y_COORD] - data.odometry_positions[Y_COORD]);
    acc_pos_new = sqrt(pow(acc_x, 2) + pow(acc_y, 2));
    acc_pos_min = acc_pos_new;
    msg.linear.x = 0.075;
    do {
        //scannedBeacon();
        //distanceToBeacon();
        acc_x = std::fabs(dest_coords[X_COORD] - data.odometry_positions[X_COORD]);
        acc_y = std::fabs(dest_coords[Y_COORD] - data.odometry_positions[Y_COORD]);
        acc_pos = (acc_x + acc_y)/2;
        acc_pos_new = sqrt(pow(acc_x, 2) + pow(acc_y, 2));
        if(acc_pos_new < acc_pos_min) acc_pos_min = acc_pos_new;
       
        msg.angular.z = angleCorrection(dest_angle, data.odometry_orientation_deg);
        //if((acc_pos_new > acc_pos_min)) {
        //    //if(acc_pos_new > acc_pos_min) break;
        //    ROS_INFO("BREAKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK");
        //    //ROS_INFO("acc_pos_new: %f", acc_pos_new);
        //    //ROS_INFO("acc_pos_min: %f", acc_pos_min);
        //    break;
        //}
        if(beaconBelow() && !distanceToBeacon()) {
            
            areal_threshold = 0.05;
            beaconDetectionTimeBased();
            appendBeaconArray();
            //break;
            adjustOrientationForDestination(dest_coords);
            msg.linear.x = 0.06;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while(time_diff < safety_duration);
            continue;
        }   
        ros::spinOnce();
        pub.publish(msg);
        //ROS_INFO("ACC_POS_NEW: %f", acc_pos_new);
        if (acc_pos_new < areal_threshold) {
            break;
        }  
    } while (true);
    ros::spinOnce();
    msg.linear.x = 0;
    msg.linear.z = 0;
    pub.publish(msg);
}

void resetBeaconDetection() {
    data.proximity_floor_values[FRONT_LEFT] = 0;
    data.proximity_floor_values[FRONT_RIGHT] = 0;
    data.proximity_floor_values[SIDE_LEFT] = 0;
    data.proximity_floor_values[SIDE_RIGHT] = 0;
}

bool beaconBelow() {
    if(data.proximity_floor_values[FRONT_LEFT] < beacon_threshold) return true;
    else if(data.proximity_floor_values[FRONT_RIGHT] < beacon_threshold) return true;
    else if(data.proximity_floor_values[SIDE_RIGHT] < beacon_threshold) return true;
    else if(data.proximity_floor_values[SIDE_LEFT] < beacon_threshold) return true;
    else return false;
}

void shortestPathCallback(const std_msgs::UInt8MultiArray& msg) {
    for(int i = 0; i <= 7; i++) {
        //data.shortest_path_order[i] = msg.data[i];
        //ROS_INFO("msg.data[i]: %i", msg.data[i]);
    }
    return;
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

    for (int idx = 0; idx < 4; ++idx) {

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
            //ROS_INFO("idx: %i", idx);
        } else {
            data.beacon_detected = false;
        }
        
        //c
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
        data.odometry_orientation_deg_encoder = 180 + (180 + y*rad2deg);
    } else {
        data.odometry_orientation_deg = y*rad2deg;
        data.odometry_orientation_deg_encoder = y*rad2deg;
    }
    data.odometry_positions_encoder[X_COORD] = msg.pose.pose.position.x;
    data.odometry_positions_encoder[Y_COORD] = msg.pose.pose.position.y;
    //data.odometry_orientation_deg_encoder = data.odometry_orientation_deg +
    //                                         (data.correction_degree * data.angle_correction_encoder);
    
    //data.odometry_positions_encoder[X_COORD] = data.odometry_positions[X_COORD];
    //data.odometry_positions_encoder[Y_COORD] = data.odometry_positions[Y_COORD];
    //ROS_INFO("x: %f, y: %f, theta: %f", data.odometry_positions[X_COORD],
    //                                    data.odometry_positions[Y_COORD], 
    //                                    data.odometry_orientation_deg);
    //ROS_INFO("x_enc: %f, y_enc: %f, theta_enc: %f", data.odometry_positions_encoder[X_COORD],
    //                                    data.odometry_positions_encoder[Y_COORD], 
    //                                    data.odometry_orientation_deg_encoder);
                                        
}

void initYAML() {
    YAML::Node beacon;

    out << YAML::BeginSeq;

    out << YAML::EndSeq;
    std::ofstream fout(yaml_path);
    fout << out.c_str();
}

void printBeaconArray() {
    for(int i = 0; i <= data.beacon_counter; i++) {
        ROS_INFO("beacon counter: %i", data.beacon_counter);
        ROS_INFO("id: %i", beacon_array[data.beacon_counter].id);
        ROS_INFO("beacon pos_x: %f", beacon_array[data.beacon_counter].pose2d[X_COORD]);
        ROS_INFO("beacon pos_y: %f", beacon_array[data.beacon_counter].pose2d[Y_COORD]);
        ROS_INFO("beacon theta: %f", beacon_array[data.beacon_counter].pose2d[THETA]);
    }
}

void appendBeaconArray() {
    ros::spinOnce();
    //int id = getBeaconId();
    //ROS_INFO("ID: %f", id);
    //scanned_beacons[data.beacon_counter][ID] = id;
    //scanned_beacons[data.beacon_counter][X_COORD] = data.odometry_positions[X_COORD];
    //scanned_beacons[data.beacon_counter][Y_COORD] = data.odometry_positions[Y_COORD];
    //scanned_beacons[data.beacon_counter][THETA] = data.odometry_orientation_deg;
    //ROS_INFO("X_COORD: %f", scanned_beacons[data.beacon_counter][X_COORD]);
    //ROS_INFO("Y_COORD: %f", scanned_beacons[data.beacon_counter][Y_COORD]);
    //ROS_INFO("THETA: %f", scanned_beacons[data.beacon_counter][THETA]);
    //ROS_INFO("ID: %f", scanned_beacons[data.beacon_counter][ID]);
    //data.beacon_counter++;
    beacon b;
    b.id = getBeaconId();
    b.pose2d[X_COORD] = data.odometry_positions[X_COORD];
    b.pose2d[Y_COORD] = data.odometry_positions[Y_COORD];
    b.pose2d[THETA] = data.odometry_orientation_deg;
    beacon_array[data.beacon_counter] = b;
    ROS_INFO("Beacon ID: %i, Beacon #: %i", beacon_array[data.beacon_counter].id, data.beacon_counter);
    ROS_INFO("X_COORD: %f Y_COORD: %f THETA: %f", beacon_array[data.beacon_counter].pose2d[X_COORD],
                                                    beacon_array[data.beacon_counter].pose2d[Y_COORD],
                                                     beacon_array[data.beacon_counter].pose2d[THETA]);
    data.beacon_counter++;
}

void getShortestPathFromYAML() {
    YAML::Node shorty = YAML::LoadFile("/home/johann/github/nightwatcher/catkin_ws/shortest_path.yaml");
    for(int i = 0; i <= 7; i++) {
        data.shortest_path_order[i] = shorty[i].as<int>();
    }
}

void appendYAML() {
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
    ros::spinOnce();
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

int shortestDistanceToBeacon() {
    int min_index;
    double pos_diff_x, pos_diff_y, distance_to_beacon;
    for(int i = 0; i < data.beacon_counter; i++) {
        pos_diff_x = beacon_array[i].pose2d[X_COORD] - data.odometry_positions[X_COORD];
        pos_diff_y = beacon_array[i].pose2d[Y_COORD] - data.odometry_positions[Y_COORD];
        distance_to_beacon = sqrt((pow(pos_diff_x, 2)) + (pow(pos_diff_y, 2)));
        distance_array[i] = distance_to_beacon;
    }
    double min = distance_array[0];
    for(int i = 0; i < data.beacon_counter; i++) {
        if(distance_array[i] < min) {
            min = distance_array[i]; 
            min_index = i;
        }
    }
    //ROS_INFO("shortest beacon: %i", min_index);
    return min_index;
}

bool distanceToBeacon() {
    ros::spinOnce();
    double x_range_high, x_range_low, y_range_high, y_range_low;
    bool isX, isY, val;
    ros::spinOnce();
    int i = shortestDistanceToBeacon();
    //ROS_INFO("beacon_array_x_coord[%i]: %f", i, beacon_array[i].pose2d[X_COORD]);
    //ROS_INFO("beacon_array_y_coord[%i]: %f", i, beacon_array[i].pose2d[Y_COORD]);
    x_range_high = beacon_array[i].pose2d[X_COORD] + 0.2;
    y_range_high = beacon_array[i].pose2d[Y_COORD] + 0.2;
    x_range_low = beacon_array[i].pose2d[X_COORD] - 0.2;
    y_range_low = beacon_array[i].pose2d[Y_COORD] - 0.2; 
    //ROS_INFO("x_range_high: %f", x_range_high);
    //ROS_INFO("y_range_high: %f", y_range_high);
    //ROS_INFO("x_range_low: %f", x_range_low);
    //ROS_INFO("y_range_low: %f", y_range_low);
    if((data.odometry_positions[X_COORD] < x_range_high) && (data.odometry_positions[X_COORD] > x_range_low)) {
        isX = true;
        //ROS_INFO("isX TRUE");
    } else {
        //ROS_INFO("isX FALSE");
        isX = false;
    }  
    if((data.odometry_positions[Y_COORD] < y_range_high) && (data.odometry_positions[Y_COORD] > y_range_low)) {
        isY = true;
        //ROS_INFO("isY TRUE");
    } else {
        isY = false;
        //ROS_INFO("isY FALSE");
    }  
    if((isX == true) && (isY == true)) {
        //ROS_INFO("VAL TRUE");
        val = true;
    } else {
        //ROS_INFO("VAL FALSE");
        val = false; 
    }
    return val;
}

bool scannedBeacon() {
    bool val;
    ros::spinOnce();
    distanceToBeacon();
    for(size_t i = 0; i < data.beacon_counter; i++) {
        if(distance_array[i] == 1) {
            val = true;
        } else {
            val = false;
        }
    }
    return val;
}

void scanBaseBeacon() {
    ros::spinOnce();
    //int id = getBeaconId();    
    //scanned_beacons[0][ID] = id;
    //scanned_beacons[0][X_COORD] = data.odometry_positions[X_COORD];
    //scanned_beacons[0][Y_COORD] = data.odometry_positions[Y_COORD];
    //scanned_beacons[0][THETA] = data.odometry_orientation_deg;
    //data.beacon_counter++;
    beacon b;
    b.id = getBeaconId();
    b.pose2d[X_COORD] = data.odometry_positions[X_COORD];
    b.pose2d[Y_COORD] = data.odometry_positions[Y_COORD];
    b.pose2d[THETA] = data.odometry_orientation_deg;
    beacon_array[0] = b;
    ROS_INFO("Beacon ID: %i, Beacon #: %i", b.id, data.beacon_counter);
    ROS_INFO("X_COORD: %f Y_COORD: %f THETA: %f", b.pose2d[X_COORD], b.pose2d[Y_COORD], b.pose2d[THETA]);
    data.beacon_counter++;
    //appendYAML(base);
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

bool proximityFrontLeftBlack() {
    if(data.proximity_floor_values[FRONT_LEFT] <= black_threshold) return true;
}

bool proximityFrontRightBlack() {
    if(data.proximity_floor_values[FRONT_RIGHT] <= black_threshold) return true;
}

bool proximityAllDetected() {
    if(data.proximity_floor_values[FRONT_LEFT] < beacon_threshold &&
        data.proximity_floor_values[FRONT_RIGHT] < beacon_threshold &&
        data.proximity_floor_values[SIDE_LEFT] < beacon_threshold &&
        data.proximity_floor_values[SIDE_RIGHT] < beacon_threshold) return true; else return false;
}

void beaconDetection() {
    geometry_msgs::Twist msg;
    double orientation_diff, orientation_diff_half, orientation_new, deg_diff;
    int first_quadrant, second_quadrant, final_quadrant, q;
    ros::Time start;
    ros::Duration time_diff;
    ros::Duration ref_duration_1(1.00);
    ros::Duration ref_duration_2(3.00);
    double dur;
    msg.linear.x = 0;
    pub.publish(msg);
    ROS_INFO("BEACON DETECTED %i", data.detecting_sensor_id);
    switch(data.detecting_sensor_id) {
        case FRONT_LEFT: {
            msg.linear.x = 0.05;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;            
            msg.angular.z = slow_rotate_cw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontRightDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            data.odometry_orientation_beacon_first_contact = data.odometry_orientation_deg;
            first_quadrant = getQuadrant(data.odometry_orientation_beacon_first_contact);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            data.odometry_orientation_beacon_second_contact = data.odometry_orientation_deg;
            second_quadrant = getQuadrant(data.odometry_orientation_beacon_second_contact);
            if(first_quadrant == quadrant_VI && second_quadrant == quadrant_I) {
                orientation_diff = data.odometry_orientation_beacon_second_contact + 
                                    (360 - data.odometry_orientation_beacon_first_contact);
            } else {
                orientation_diff = (data.odometry_orientation_beacon_second_contact -
                                    data.odometry_orientation_beacon_first_contact);
            }
            orientation_diff_half = orientation_diff / 2;
            orientation_new = data.odometry_orientation_beacon_second_contact - 
                                orientation_diff_half;
            if(orientation_new < 0) { 
                orientation_new = 360 + orientation_new;
            }
            ROS_INFO("first: %f", data.odometry_orientation_beacon_first_contact);
            ROS_INFO("second: %f", data.odometry_orientation_beacon_second_contact);
            ROS_INFO("or_new: %f", orientation_new);
            final_quadrant = getQuadrant(orientation_new);
            msg.angular.z = slow_rotate_cw;
            pub.publish(msg);
            if(second_quadrant == quadrant_I && final_quadrant == quadrant_VI) {
                if((orientation_new < 0.5) || ((360 - orientation_new) < 0.5)) {
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(q != quadrant_VI);    
                } else {
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(q != quadrant_VI);
                    do {
                        deg_diff = data.odometry_orientation_deg - orientation_new;
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(deg_diff > accuracyConstAngle);
                }
            } else {
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
                    } while(deg_diff > accuracyConstAngle);
                }
            }
            msg.angular.z = 0;
            msg.linear.x = -0.02;
            pub.publish(msg);
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
            msg.angular.z = 0.03;
            ros::Duration(0.5).sleep();
            msg.linear.x = 0;
            msg.angular.z = 0;
            pub.publish(msg);
            appendBeaconArray();
            break;
        }
        case FRONT_RIGHT: {
            ROS_INFO("FRONTRIGHT");
            msg.linear.x = 0.05;
            pub.publish(msg);
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            data.odometry_orientation_beacon_first_contact = data.odometry_orientation_deg;
            first_quadrant = getQuadrant(data.odometry_orientation_beacon_first_contact);
            msg.angular.z = slow_rotate_cw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontRightDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            data.odometry_orientation_beacon_second_contact = data.odometry_orientation_deg;
            second_quadrant = getQuadrant(data.odometry_orientation_beacon_second_contact);
            if(first_quadrant == quadrant_I && second_quadrant == quadrant_VI) {
                orientation_diff = data.odometry_orientation_beacon_first_contact +
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
            ROS_INFO("orientation_new: %f", orientation_new);
            final_quadrant = getQuadrant(data.odometry_orientation_deg);
            msg.angular.z = slow_rotate_ccw;
            pub.publish(msg);
            if(second_quadrant == quadrant_VI && final_quadrant == quadrant_I) {
                if((orientation_new < 0.5) || (360 - orientation_new)) {
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while (q != quadrant_I);
                } else {
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
                }
            } else {
                if(orientation_new < 0.5){
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while (q != quadrant_I);
                } else {
                    do {
                        deg_diff = orientation_new - data.odometry_orientation_deg;
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(deg_diff > accuracyConstAngle);
                }
            }
            msg.angular.z = 0;
            msg.linear.x = -0.02;
            pub.publish(msg);
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected() && proximityFrontRightDetected());
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
            msg.angular.z = 0.03;
            ros::Duration(0.5).sleep();
            msg.linear.x = 0;
            msg.angular.z = 0;
            appendBeaconArray();
            break;
            
        }
        case SIDE_LEFT: {
            ROS_INFO("SIDE_LEFT");
            msg.linear.x = 0.05;
            msg.angular.z = -0.5;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;

            msg.angular.z = slow_rotate_cw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while (proximityFrontRightDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            data.odometry_orientation_beacon_first_contact = data.odometry_orientation_deg;
            first_quadrant = getQuadrant(data.odometry_orientation_beacon_first_contact);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            data.odometry_orientation_beacon_second_contact = data.odometry_orientation_deg;
            second_quadrant = getQuadrant(data.odometry_orientation_beacon_second_contact);
            if (first_quadrant == quadrant_VI && second_quadrant == quadrant_I) {
                orientation_diff = data.odometry_orientation_beacon_second_contact +
                                   (360 - data.odometry_orientation_beacon_first_contact);
            } else {
                orientation_diff = (data.odometry_orientation_beacon_second_contact -
                                    data.odometry_orientation_beacon_first_contact);
                if(orientation_diff < 0) {
                    orientation_diff = (360 - data.odometry_orientation_beacon_second_contact -
                                        data.odometry_orientation_beacon_first_contact);
                }
            }
            orientation_diff_half = orientation_diff / 2;
            orientation_new = data.odometry_orientation_beacon_second_contact -
                              orientation_diff_half;
            if (orientation_new < 0) {
                orientation_new = 360 + orientation_new;
            }
            ROS_INFO("first: %f", data.odometry_orientation_beacon_first_contact);
            ROS_INFO("second: %f", data.odometry_orientation_beacon_second_contact);
            ROS_INFO("or_new: %f", orientation_new);
            final_quadrant = getQuadrant(orientation_new);
            msg.angular.z = slow_rotate_cw;
            pub.publish(msg);
            if (second_quadrant == quadrant_I && final_quadrant == quadrant_VI) {
                if ((orientation_new < 0.5) || ((360 - orientation_new) < 0.5)) {
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(q != quadrant_VI);
                } else {
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(q != quadrant_VI);
                    do {
                        deg_diff = data.odometry_orientation_deg - orientation_new;
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(deg_diff > accuracyConstAngle);
                }
            } else {
                if ((orientation_new < 0.5)) {
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
                    } while(deg_diff > accuracyConstAngle);
                }
            }
            msg.angular.z = 0;
            msg.linear.x = -0.02;
            pub.publish(msg);
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
            msg.angular.z = 0.03;
            ros::Duration(0.5).sleep();
            msg.linear.x = 0;
            msg.angular.z = 0;
            pub.publish(msg);
            appendBeaconArray();
            break;
        }
        case SIDE_RIGHT: {
            ROS_INFO("SIDE_RIGHT");
            msg.linear.x = 0.05;
            msg.angular.z = 0.5;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while(time_diff < ref_duration_1);
            msg.linear.x = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontLeftDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            data.odometry_orientation_beacon_first_contact = data.odometry_orientation_deg;
            first_quadrant = getQuadrant(data.odometry_orientation_beacon_first_contact);
            msg.angular.z = slow_rotate_cw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while(proximityFrontRightDetected());
            msg.angular.z = 0;
            pub.publish(msg);
            data.odometry_orientation_beacon_second_contact = data.odometry_orientation_deg;
            second_quadrant = getQuadrant(data.odometry_orientation_beacon_second_contact);
            ROS_INFO("fq: %i", first_quadrant);
            ROS_INFO("sq: %i", second_quadrant);
            if(first_quadrant == quadrant_I && second_quadrant == quadrant_VI) {
                ROS_INFO("IF");
                ROS_INFO("or_diff: %f", orientation_diff);
                orientation_diff = data.odometry_orientation_beacon_first_contact +
                                   (360 - data.odometry_orientation_beacon_second_contact);
            } else {
                ROS_INFO("ELSE");
                ROS_INFO("or_diff: %f", orientation_diff);
                orientation_diff = data.odometry_orientation_beacon_first_contact -
                                   data.odometry_orientation_beacon_second_contact;
                if(orientation_diff < 0) {
                    orientation_diff = (360 - data.odometry_orientation_beacon_first_contact) -
                                       data.odometry_orientation_beacon_second_contact;
                }
            }
            orientation_diff_half = orientation_diff / 2;
            orientation_new = data.odometry_orientation_beacon_first_contact -
                              orientation_diff_half;
            if(orientation_new < 0) {
                orientation_new = 360 + orientation_new;
            }
            ROS_INFO("orientation_new: %f", orientation_new);
            ROS_INFO("first %f", data.odometry_orientation_beacon_first_contact);
            ROS_INFO("second: %f", data.odometry_orientation_beacon_second_contact);
            final_quadrant = getQuadrant(orientation_new);
            msg.angular.z = slow_rotate_ccw;
            pub.publish(msg);
            if(second_quadrant == quadrant_VI && final_quadrant == quadrant_I) {
                if((orientation_new < 0.5) || (360 - orientation_new)) {
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(q != quadrant_I);
                } else {
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
                }
            } else {
                if(orientation_new < 0.5) {
                    do {
                        q = getQuadrant(data.odometry_orientation_deg);
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(q != quadrant_I);
                } else {
                    do {
                        deg_diff = orientation_new - data.odometry_orientation_deg;
                        pub.publish(msg);
                        ros::spinOnce();
                    } while(deg_diff > accuracyConstAngle);
                }
            }
            msg.angular.z = 0;
            msg.linear.x = -0.02;
            pub.publish(msg);
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while (proximityFrontLeftDetected() && proximityFrontRightDetected());
            msg.linear.x = 0;
            pub.publish(msg);
            msg.linear.x = 0.035;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while (time_diff < ref_duration_2);
            msg.linear.x = 0;
            pub.publish(msg);
            msg.angular.z = slow_rotate_ccw;
            do {
                pub.publish(msg);
                ros::spinOnce();
            } while (!proximityFrontBlack());
            msg.angular.z = 0.03;
            ros::Duration(0.5).sleep();
            msg.linear.x = 0;
            msg.angular.z = 0;
            appendBeaconArray();
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
    
    do {  
        if(!even_step) {    
            next_hop_x = last_hop_x + (even_step_counter * amiro_base_diameter * uneven_sign);
            next_hop_y = last_hop_y;
            even_step = true;
            uneven_step_counter++;
            uneven_sign *= (-1);
            next_hop_coords[X_COORD] = next_hop_x;
            next_hop_coords[Y_COORD] = next_hop_y;
            ROS_INFO("%i. HOP: %f, %f", data.hop, next_hop_x, next_hop_y);
            moveToCoordinates(next_hop_coords);
            last_hop_x = next_hop_x;
            last_hop_y = next_hop_y;
            data.hop++;
        } else {
            next_hop_x = last_hop_x;
            next_hop_y = last_hop_y + (even_step_counter * amiro_base_diameter * even_sign);
            even_step = false;
            even_step_counter++;
            even_sign *= (-1);
            next_hop_coords[X_COORD] = next_hop_x;
            next_hop_coords[Y_COORD] = next_hop_y;
            ROS_INFO("%i. HOP: %f, %f", data.hop, next_hop_x, next_hop_y);
            moveToCoordinates(next_hop_coords);
            last_hop_x = next_hop_x;
            last_hop_y = next_hop_y;
            data.hop++;
        }
        
    } while(data.beacon_counter < 7);
}

double adjustOrientationForDestinationEncoder(double dest_coords[]) {
    geometry_msgs::Twist msg;
    double deg_diff = 0;
    bool acc_deg = false;
    double position_offset_x = dest_coords[X_COORD] - data.odometry_positions_encoder[X_COORD];
    double position_offset_y = dest_coords[Y_COORD] - data.odometry_positions_encoder[Y_COORD];
    double dest_angle = rad2deg * calculateAngle(position_offset_x, position_offset_y);
    
    if (dest_angle < 0) {
        dest_angle = 180 + (180 + dest_angle);
    } else dest_angle = dest_angle;
    dest_angle = rotateToDestinationEncoder(dest_angle, data.odometry_orientation_deg_encoder);
    msg.angular.z = 0;
    pub.publish(msg);
    ros::spinOnce();
    return dest_angle;
}

bool rescueMode() {
    geometry_msgs::Twist msg;
    ros::Duration time_diff;
    ros::Duration ref_duration(5.00);
    ros::Time start;
    double vel = 0.06;
    double rescue_rotate_ccw = 0.15;
    int counter = 0;
    msg.linear.x = vel;
    do {
        start = ros::Time::now();
        do {
            pub.publish(msg);
            ros::spinOnce();
            if(beaconBelow() && !scannedBeacon()) {
                beaconDetectionTimeBased();
                return true;
            }
            time_diff = ros::Time::now() - start;
        } while(time_diff < ref_duration);
        if(beaconBelow() && !scannedBeacon()) {
            beaconDetectionTimeBased();
            return true;
        } 
        msg.linear.x = -vel;
        start = ros::Time::now();
        do {
            pub.publish(msg);
            ros::spinOnce();
            if(beaconBelow() && !scannedBeacon()) {
                beaconDetectionTimeBased();
                return true;
            } 
            time_diff = ros::Time::now() - start;
        } while(time_diff < ref_duration);
        
        msg.linear.x = 0;
        msg.angular.z = rescue_rotate_ccw;
        start = ros::Time::now();
        do {
            pub.publish(msg);
            ros::spinOnce();
            if(beaconBelow() && !scannedBeacon()) {
                beaconDetectionTimeBased();
                return true;
            }
            time_diff = ros::Time::now() - start;
        } while(time_diff < ref_duration);
        
        msg.linear.x = vel;
        msg.angular.z = 0;
        counter++;
    } while(counter < 16);
    return false;
}

void printShortestPathOrder() {
    for(size_t i = 0; i < 8; i++) {
        for(size_t j = 0; j < 3; j++) {
            ROS_INFO("beacon: %i: %f", data.shortest_path_order[i], beacon_vector[data.shortest_path_order[i]][j]);
        }
    }
}

bool scannedBeaconEncoder() {
    ros::spinOnce();
    double pos_diff_x, pos_diff_y, distance_to_beacon;
    for(int i = 0; i <= data.beacon_counter; i++) {
        pos_diff_x = beacon_array[i].pose2d[Y_COORD] - data.odometry_positions_encoder[Y_COORD];
        pos_diff_y = beacon_array[i].pose2d[X_COORD] - data.odometry_positions_encoder[X_COORD];
        distance_to_beacon = sqrt((pow(pos_diff_x, 2)) + (pow(pos_diff_y, 2)));
        //ROS_INFO("i = %i , distance: %f", i, distance_to_beacon);
        if(distance_to_beacon < 0.2) {
            //ROS_INFO("TRUE");
            return true;
        } else {
            //ROS_INFO("FALSE");
            return false;
        }
    }
}

void calculateError(bool beacon_found, std::vector<double> actual_data) {
    ros::spinOnce();
    if(beacon_found) {
        data.offset_after_calibration = (actual_data[THETA]) - data.odometry_orientation_deg_encoder;
        data.encoder_position_error_x = (actual_data[X_COORD] - 2.5) - data.odometry_positions_encoder[X_COORD];
        data.encoder_position_error_y = (actual_data[Y_COORD] - 2.5) - data.odometry_positions_encoder[Y_COORD];
        //data.angular_error_per_degree = data.angular_error_per_degree * (1.2);
        //data.offset_after_calibration = data.odometry_orientation_deg_encoder - beacon_vector[data.shortest_path_order[data.order_counter+1]][THETA];
        //data.encoder_position_error_x = data.odometry_positions_encoder[X_COORD] - beacon_vector[data.shortest_path_order[data.beacon_counter]+1][X_COORD];
        //data.encoder_position_error_x = data.odometry_positions_encoder[Y_COORD] - beacon_vector[data.shortest_path_order[data.beacon_counter]+1][Y_COORD];
        //data.offset_after_calibration = beacon_vector[data.shortest_path_order[data.beacon_counter]][THETA] - data.odometry_orientation_deg_encoder;
        //data.encoder_position_error_x = beacon_vector[data.shortest_path_order[data.beacon_counter]][X_COORD] - data.odometry_positions_encoder[X_COORD];
        //data.encoder_position_error_x = beacon_vector[data.shortest_path_order[data.beacon_counter]][Y_COORD] - data.odometry_positions_encoder[Y_COORD];
        //ROS_INFO("EXPECTED THETA: %f", beacon_vector[data.shortest_path_order[data.order_counter]+1][THETA]);
        ROS_INFO("X_ERROR: %f = %f - %f", data.encoder_position_error_x, (actual_data[X_COORD] - 2.5), data.odometry_positions_encoder[X_COORD]);
        ROS_INFO("Y_ERROR: %f = %f - %f", data.encoder_position_error_y, (actual_data[Y_COORD]- 2.5), data.odometry_positions_encoder[Y_COORD]);
        ROS_INFO("THETA ERROR: %f = %f - %f", data.offset_after_calibration, actual_data[THETA], data.odometry_orientation_deg_encoder);
    } else {
        data.offset_after_calibration = angular_error_after_failed_rescue;
    }
}

double adjustOrientationEncoder(double dest_coords[]) {
    geometry_msgs::Twist msg;
    double offset_route_0 = 2.5; // NOTE Please adjust according to each Map!
    msg.linear.x = 0;
    msg.angular.z = 0;
    pub.publish(msg);
    dest_coords[X_COORD] = (dest_coords[X_COORD] - offset_route_0) /*+ data.encoder_position_error_x*/;
    dest_coords[Y_COORD] = (dest_coords[Y_COORD] - offset_route_0) /*+ data.encoder_position_error_y*/;
    double dest_angle = adjustOrientationForDestinationEncoder(dest_coords);
    return dest_angle;
}

void moveToPositionEncoder(double *dest_coords, double dest_angle, std::vector<double> actual_data) {
    geometry_msgs::Twist msg;
    ros::Duration time_diff;
    ros::Duration safety_duration(1.00);
    ros::Time start;
    double acc_x = 0;
    double acc_y = 0;
    double acc_pos = 0;
    double acc_pos_help[2], acc_pos_min;
    bool overshoot;
    double offset_route_0 = 2.5; // NOTE Please adjust according to each Map!
    ROS_INFO("dest_coords: %f %f", dest_coords[X_COORD], dest_coords[Y_COORD]);
    dest_coords[X_COORD] = (dest_coords[X_COORD]) /*+ data.encoder_position_error_x*/;
    dest_coords[Y_COORD] = (dest_coords[Y_COORD]) /*+ data.encoder_position_error_y*/;
    acc_x = (dest_coords[X_COORD] - data.odometry_positions_encoder[X_COORD]);
    acc_y = (dest_coords[Y_COORD] - data.odometry_positions_encoder[Y_COORD]);
    acc_pos = sqrt(pow(acc_x, 2) + pow(acc_y, 2));
    acc_pos_min = acc_pos;
    msg.linear.x = 0.075;
    pub.publish(msg);
    start = ros::Time::now();
    do {
        pub.publish(msg);
        ros::spinOnce();
        time_diff = ros::Time::now() - start;
    } while(time_diff < safety_duration);
    do {
        pub.publish(msg);
        ros::spinOnce();
        acc_x = (dest_coords[X_COORD] - data.odometry_positions_encoder[X_COORD]);
        acc_y = (dest_coords[Y_COORD] - data.odometry_positions_encoder[Y_COORD]);
        acc_pos = sqrt(pow(acc_x, 2) + pow(acc_y, 2));
        //ROS_INFO("acc_pos_min: %f", acc_pos_min);
        if(acc_pos < acc_pos_min) {
            acc_pos_min = acc_pos;
        } 
        //ROS_INFO("acc_pos: %f", acc_pos);
        msg.angular.z = angleCorrection(dest_angle, data.odometry_orientation_deg_encoder);
        if (beaconBelow() && !scannedBeaconEncoder() && acc_pos > 0.1) {
            scannedBeacon();
            beaconDetectionTimeBased();
            appendBeaconArray();
            calculateError(BEACON_FOUND, actual_data);
            break;
        } else if(acc_pos > acc_pos_min) {
            msg.linear.x = 0;
            pub.publish(msg);
            if(rescueMode()) {
                appendBeaconArray();
                calculateError(BEACON_FOUND, actual_data);
                break;
            } else {
                calculateError(NO_BEACON_FOUND, actual_data);
                break;
            }
        }
    } while(true);
}

void moveToCoordinatesEncoder(double *dest_coords) {
    geometry_msgs::Twist msg;
    ros::Duration time_diff;
    ros::Duration safety_duration(2.00);
    ros::Time start;
    double acc_x = 0;
    double acc_y = 0;
    double acc_pos = 0;
    double offset_route_0 = 2.5; // NOTE Please adjust according to each Map! 
    double next_coords[2] = {0, 0};
    dest_coords[X_COORD] = dest_coords[X_COORD] - offset_route_0 /*+ data.encoder_position_error_x*/;
    dest_coords[Y_COORD] = dest_coords[Y_COORD] - offset_route_0 /*+ data.encoder_position_error_y*/;
    double dest_angle = adjustOrientationForDestinationEncoder(dest_coords);
    ROS_INFO("dest_angle for correction: %f", dest_angle);
    ROS_INFO("dest_coords: [%f][%f]", dest_coords[X_COORD], dest_coords[Y_COORD]);

    msg.linear.x = 0.075;
    pub.publish(msg);
    do {
        //acc_x = std::fabs(dest_coords[X_COORD] - (data.odometry_positions_encoder[X_COORD]));
        //acc_y = std::fabs(dest_coords[Y_COORD] - (data.odometry_positions_encoder[Y_COORD]));
        //acc_pos = (acc_x + acc_y) / 2;
        acc_x = (dest_coords[X_COORD] - data.odometry_positions_encoder[X_COORD]);
        acc_y = (dest_coords[Y_COORD] - data.odometry_positions_encoder[Y_COORD]);
        acc_pos = sqrt(pow(acc_x, 2) + pow(acc_y, 2));
        //ROS_INFO("acc_pos: %f", acc_pos);
        msg.angular.z = angleCorrection(dest_angle, data.odometry_orientation_deg_encoder);
        if (beaconBelow() && !distanceToBeacon() && acc_pos > 0.1) {
            beaconDetectionTimeBased();
            appendBeaconArray();
            //calculateError(BEACON_FOUND);
            next_coords[X_COORD] = beacon_vector[data.shortest_path_order[data.order_counter]+1][X_COORD] - offset_route_0;
            next_coords[Y_COORD] = beacon_vector[data.shortest_path_order[data.order_counter]+1][Y_COORD] - offset_route_0;
            ROS_INFO("next_coords: [%f][%f]", next_coords[X_COORD], next_coords[Y_COORD]);
            ROS_INFO("THETA_ACT: %f", data.odometry_orientation_deg_encoder);
            ROS_INFO("X_COORD_ACT: %f", data.odometry_positions_encoder[X_COORD]);
            ROS_INFO("Y_COORD_ACT: %f", data.odometry_positions_encoder[Y_COORD]);
            adjustOrientationForDestinationEncoder(next_coords);
            
            msg.linear.x = 0.075;
            start = ros::Time::now();
            do {
                pub.publish(msg);
                ros::spinOnce();
                time_diff = ros::Time::now() - start;
            } while (time_diff < safety_duration);
            //break;
        } else if(!beaconBelow() && !scannedBeacon() && acc_pos < 0.1){
            ros::spinOnce();
            ROS_INFO("NO BEACON FOUND");
            msg.linear.x = 0;
            pub.publish(msg);
            rescueMode();
            appendBeaconArray();
            //calculateError(NO_BEACON_FOUND);
            break;
        }
        ros::spinOnce();
        pub.publish(msg);
        //if (acc_pos < accuracyConstPosition) break; 
        //break;
    } while(true);
    ros::spinOnce();
    msg.linear.x = 0;
    msg.linear.z = 0;
    pub.publish(msg);
}

void exportAsYAML() {
    YAML::Node shortest_path;
	YAML::Emitter out;
    for(int i = 0; i <= data.beacon_counter; i++) {
	    out << YAML::Flow;
	    out << YAML::BeginSeq << beacon_array[i].id << beacon_array[i].pose2d[X_COORD] << beacon_array[i].pose2d[Y_COORD] << beacon_array[i].pose2d[THETA] << YAML::EndSeq;
    }
	std::ofstream fout("scanned_beacons.yaml");
	fout << out.c_str();
}

void exploreMap() {
    getInitPosition();
    scanBaseBeacon();
    scannedBeacon();
    exploration();
    exportAsYAML();
}

void beaconNavigation() {
    ros::Duration time_diff;
    ros::Duration safety_duration(1.00);
    ros::Time start;
    geometry_msgs::Twist msg;
    double dest_coords[2] = {0, 0};
    double dest_angle;
    //getShortestPathFromYAML();
    scanBaseBeacon();
    scannedBeacon();
    //for(size_t i = 1; i <= 1; i++) {
    //    ROS_INFO("b: %i", data.shortest_path_order[i]);
    //    dest_coords[X_COORD] = beacon_vector[data.shortest_path_order[i]][X_COORD];
    //    dest_coords[Y_COORD] = beacon_vector[data.shortest_path_order[i]][Y_COORD];
    //    moveToCoordinatesEncoder(dest_coords);
    //    //data.beacon_counter++;
    //}
    // TODO make it some sort of dynamic
    // NOTE this represents the shortest path calculated by the shortest_path pkg
    double dest_coords_1[2] = {2.5, 3.0};
    double dest_coords_2[2] = {3.25, 3.0};
    double dest_coords_3[2] = {3.5, 2.5};
    double dest_coords_4[2] = {1.75, 3.00};
    double dest_coords_5[2] = {1.50, 2.5};
    double dest_coords_6[2] = {2.5, 2.00};
    double dest_coords_7[2] = {2.5, 2.5};
    dest_angle = adjustOrientationEncoder(dest_coords_1);
    moveToPositionEncoder(dest_coords_1, dest_angle, beacon_1);
    dest_angle = adjustOrientationEncoder(dest_coords_2);
    moveToPositionEncoder(dest_coords_2, dest_angle, beacon_2);
    dest_angle = adjustOrientationEncoder(dest_coords_3);
    moveToPositionEncoder(dest_coords_3, dest_angle, beacon_0);
    dest_angle = adjustOrientationEncoder(dest_coords_4);
    moveToPositionEncoder(dest_coords_4, dest_angle, beacon_3);
    dest_angle = adjustOrientationEncoder(dest_coords_5);
    moveToPositionEncoder(dest_coords_5, dest_angle, beacon_4);
    dest_angle = adjustOrientationEncoder(dest_coords_6);
    moveToPositionEncoder(dest_coords_6, dest_angle, beacon_5);
    dest_angle = adjustOrientationEncoder(dest_coords_7);
    moveToPositionEncoder(dest_coords_7, dest_angle, base);
    //msg.linear.x = 0.075;
    //start = ros::Time::now();
    //do {
    //    pub.publish(msg);
    //    ros::spinOnce();
    //    time_diff = ros::Time::now() - start;
    //} while(time_diff < safety_duration);

    //rescueMode();
    //moveToCoordinatesEncoder(dest_coords_2);
    //moveToCoordinatesEncoder(dest_coords_3);
    //double dest_coords_4[2] = {2.5, 2.5};
    //moveToCoordinatesEncoder(dest_coords_4);
    //double dest_coords_next[2] = {3, 3};
    //adjustOrientationForDestination(dest_coords_next);
    //moveToCoordinatesEncoder(dest_coords_next);
}

main(int argc, char **argv) {
    ros::init(argc, argv, "nav");
    
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Twist>("amiro1/cmd_vel", 1);
    ros::Subscriber odometry_sub = n.subscribe("amiro1/odom", 1, odometryDataCallback);
    ros::Subscriber shortest_path_sub = n.subscribe("/shortest_path", 100, shortestPathCallback);
    ros::Rate loop_rate(50);
    geometry_msgs::Twist msg;
    std::string topic_out, topic_in_suffix, topic_in_prefix;

    // NOTE change routes here!
    n.getParam("/watchmen_route_0/base/pose2d", base);
    n.getParam("/watchmen_route_0/beacon_0/pose2d", beacon_0);
    n.getParam("/watchmen_route_0/beacon_1/pose2d", beacon_1);
    n.getParam("/watchmen_route_0/beacon_2/pose2d", beacon_2);
    n.getParam("/watchmen_route_0/beacon_3/pose2d", beacon_3);
    n.getParam("/watchmen_route_0/beacon_4/pose2d", beacon_4);
    n.getParam("/watchmen_route_0/beacon_5/pose2d", beacon_5);
    
    beacon_vector.push_back(base);
    beacon_vector.push_back(beacon_0);
    beacon_vector.push_back(beacon_1);
    beacon_vector.push_back(beacon_2);
    beacon_vector.push_back(beacon_3);
    beacon_vector.push_back(beacon_4);
    beacon_vector.push_back(beacon_5);

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
    
    
    
    
    ros::Duration(3.0).sleep();
    
    
    //exploreMap(); //NOTE Use World Odometry 
    beaconNavigation(); //NOTE Use Encoder Odometry
    return 0;
}
