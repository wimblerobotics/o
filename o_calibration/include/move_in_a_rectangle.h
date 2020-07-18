#ifndef __MOVE_IN_A_RECTANGLE__
#define __MOVE_IN_A_RECTANGLE__

#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

class MoveInARectangle {
public:
    MoveInARectangle(ros::NodeHandle& nh);

	bool init();

	bool run();

private:
    typedef enum STATE {
        kDONE,
        kFORWARD,
        kROTATE_RIGHT,
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
    
    u_long last_odometry_msg_counter_;
    nav_msgs::Odometry last_odometry_msg_;

    u_long t265_last_odometry_msg_counter_;
    nav_msgs::Odometry t265_last_odometry_msg_;

    // Class variables.
    double goal_x_;
    double goal_z_;
    nav_msgs::Odometry start_odometry_;
    bool start_odometry_found_;
    nav_msgs::Odometry t265_start_odometry_;
    bool t265_start_odometry_found_;
    STATE state_;

    // Functions.
    double normalizeRadians(double theta);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void t265OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    std::string eulerString(const sensor_msgs::Imu_<std::allocator<void> >::_orientation_type& q, u_long counter);
    void report();
    void resetEncoders();

};

#endif
