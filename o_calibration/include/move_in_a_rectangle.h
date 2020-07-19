#ifndef __MOVE_IN_A_RECTANGLE__
#define __MOVE_IN_A_RECTANGLE__

#include <ros/ros.h>
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
// #include <visualization_msgs/Marker.h>

class MoveInARectangle {
public:
    MoveInARectangle(ros::NodeHandle& nh);

	bool init();

	bool run();

    static bool odomMessagesReceived() {
        return last_odometry_msg_counter_ != 0;
    }

    static bool t265OdomMessagesReceived() {
        return t265_last_odometry_msg_counter_ != 0;
    }

private:
    typedef enum STATE {
        kDONE,
        kFORWARD,
        kROTATE_LEFT,
        KSTART,
    } STATE;

	// ROS Parameters.

	// ROS variables;
	ros::NodeHandle& nh_;					    // ROS node handle;
	ros::Time prev_;						    // Previous loop time, for computing durations.
    ros::ServiceClient resetEncodersService_;   // For reseting the encoders.
 
    ros::Publisher cmd_vel_publisher_;          // Publisher for geometry_msgs/Twist
    ros::Subscriber odometry_subscriber_;       // Subscriber to nav_msgs/odometry message.
    ros::Subscriber t265_odometry_subscriber_;  // Subscriber to nav_msgs/odometry message.

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    
    static u_long last_odometry_msg_counter_;
    nav_msgs::Odometry last_odometry_msg_;

    static u_long t265_last_odometry_msg_counter_;
    nav_msgs::Odometry t265_last_odometry_msg_;

    // Class variables.
    double step_x_;
    double step_z_;
    nav_msgs::Odometry start_odometry_;
    bool start_odometry_found_;
    nav_msgs::Odometry t265_start_odometry_;
    bool t265_start_odometry_found_;
    STATE state_;

    // Functions.
    double normalizeRadians(double theta);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void t265OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    std::string eulerString(const geometry_msgs::Quaternion& q, u_long counter);
    std::string getStateString(STATE state);
    void report();
    std::string reportOdom(bool odometry_found, nav_msgs::Odometry odometry, std::string prefix);
    void resetEncoders();

};

#endif
