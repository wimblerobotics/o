#include "ros/ros.h"
#include "move_in_a_rectangle.h"

#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <iomanip>      // std::setprecision
#include <iostream>
#include <o_hardware/ResetEncoders.h>
#include <ros/console.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <vector>

MoveInARectangle::MoveInARectangle(ros::NodeHandle& nh)
	: goal_x_(1.0)
    ,  goal_z_(M_PI)
    , last_odometry_msg_counter_(0)
    , t265_last_odometry_msg_counter_(0)
    , nh_(nh)
    , prev_(ros::Time::now())
    , start_odometry_found_(false)
    , t265_start_odometry_found_(false)
    , state_(KSTART) {
    resetEncodersService_ = nh.serviceClient<o_hardware::ResetEncoders>("reset_encoders");
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("/o/diff_drive_controller/cmd_vel", 1);
    odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("o/diff_drive_controller/odom", 10, boost::bind(&MoveInARectangle::odometryCallback, this, _1));
    t265_odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 10, boost::bind(&MoveInARectangle::t265OdometryCallback, this, _1));
}


double MoveInARectangle::normalizeRadians(double theta) {
    double TWO_PI = 2 * M_PI;
    double normalized = fmod(theta, TWO_PI);
    normalized = fmod((normalized + TWO_PI), TWO_PI);
    return normalized <= M_PI ? normalized : normalized - TWO_PI;
}

bool MoveInARectangle::init() {
    ROS_INFO("[MoveInARectangle::init]");
    return true;
}

void MoveInARectangle::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    last_odometry_msg_ = *msg;
    last_odometry_msg_counter_++;
    if (!start_odometry_found_) {
        start_odometry_ = *msg;
        start_odometry_found_ = true;
    }
    //ROS_INFO("[MoveInARectangle::odometryCallback] message received counter:: %lu", last_odometry_msg_counter_);
}


void MoveInARectangle::t265OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    t265_last_odometry_msg_ = *msg;
    t265_last_odometry_msg_counter_++;
    if (!t265_start_odometry_found_) {
        t265_start_odometry_ = *msg;
        t265_start_odometry_found_ = true;
    }
    //ROS_INFO("[MoveInARectangle::t265OdometryCallback] message received counter:: %lu", last_odometry_msg_counter_);
}


std::string MoveInARectangle::eulerString(const sensor_msgs::Imu_<std::allocator<void> >::_orientation_type& q, u_long counter) {
    tf::Quaternion qq(q.x, q.y, q.z, q.w);
    std::stringstream s;
    s << "angle: " << std::setprecision(4);
    if (counter > 0) {
        double yaw = tf::getYaw(qq);
        s << yaw;
        s << "r (" << ((yaw * 360) / (2 * M_PI)) << "d)";
        s << std::setprecision(4);
        s << ", QUAT x: " << q.x;
        s << ", y: " << q.y;
        s << ", z: " << q.z;
        s << ", w: " << q.w;
    } else {
        s << "NO DATA";
    }
    
    return s.str();
}


void MoveInARectangle::report() {
    std::stringstream s;
    std::string calibStatus = "??";

    s << "  ODOM angle: " << std::setprecision(2) << eulerString(last_odometry_msg_.pose.pose.orientation, last_odometry_msg_counter_) << std::endl;
    if (last_odometry_msg_counter_ > 0) {
        s << "  ODOM pos x: " << std::setprecision(4) 
        << last_odometry_msg_.pose.pose.position.x 
        << ", y: " 
        << last_odometry_msg_.pose.pose.position.y 
        << ", z: "
        << last_odometry_msg_.pose.pose.position.z;
    } else {
        s << "  ODOM NO DATA";
    }
    s  << std::endl;


    s << "  t265 ODOM angle: " << std::setprecision(2) << eulerString(t265_last_odometry_msg_.pose.pose.orientation, t265_last_odometry_msg_counter_) << std::endl;
    if (t265_last_odometry_msg_counter_ > 0) {
        s << "  ODOM pos x: " << std::setprecision(4) 
        << t265_last_odometry_msg_.pose.pose.position.x 
        << ", y: " 
        << t265_last_odometry_msg_.pose.pose.position.y 
        << ", z: "
        << t265_last_odometry_msg_.pose.pose.position.z;
    } else {
        s << "  ODOM NO DATA";
    }
    s  << std::endl;


    ROS_INFO_STREAM("[MoveInARectangle::report] "  << s.str());
}



bool MoveInARectangle::run() {
    std::string state_string = "????";
    double start_yaw = 0.0;
    switch (state_) {
        case kDONE: state_string = "DONE"; break;
        case kFORWARD: state_string = "kFORWARD"; break;
        case kROTATE_RIGHT: state_string = "kROTATE_RIGHT"; break;
        case KSTART: state_string = "KSTART"; break;
    }

    std::stringstream start_goal_string;
    if (start_odometry_found_ > 0) {
        tf::Quaternion qq(start_odometry_.pose.pose.orientation.x,
                          start_odometry_.pose.pose.orientation.y, 
                          start_odometry_.pose.pose.orientation.z, 
                          start_odometry_.pose.pose.orientation.w);
        start_yaw = tf::getYaw(qq);
        start_goal_string << std::setprecision(4);
        start_goal_string << "start x: " << start_odometry_.pose.pose.position.x;
        start_goal_string << ", y: " << start_odometry_.pose.pose.position.y;
        start_goal_string << ", z: " << start_odometry_.pose.pose.position.z;
        start_goal_string << ", start_yaw: " << start_yaw << "r (" << ((start_yaw * 360) / (2 * M_PI)) << "d)";
    } else {
        start_goal_string << "NO ODOM";
    }

    ROS_INFO_STREAM("[MoveInARectangle::run] state: "
                    << state_string
                    << start_goal_string.str());

    switch (state_) {
        case kDONE:
            return false;
            break;
        
        case kFORWARD:
            if (last_odometry_msg_counter_ > 0) {
                double goal_x = start_odometry_.pose.pose.position.x + goal_x_;
                double goal_x_to_go =  goal_x - last_odometry_msg_.pose.pose.position.x;
                ROS_INFO("[MoveInARectangle::run] kFORWARD goal x: %7.4f, x travel to go %7.4f m", 
                         goal_x,
                         goal_x_to_go);
                if (goal_x_to_go > 0) {
                    ROS_INFO("[MoveInARectangle::run] kFORWARD still need to travel forward");
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.1;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_publisher_.publish(cmd_vel);
                } else {
                    // STOP
                    double odometry_x_moved = last_odometry_msg_.pose.pose.position.x - start_odometry_.pose.pose.position.x;
                    state_ = kROTATE_RIGHT;
                    ROS_INFO("[MoveInARectangle::run] kFORWARD goal reached, begin rotation. Odometry distance moved: %7.4f.", odometry_x_moved);
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_publisher_.publish(cmd_vel);
                    resetEncoders();
                }
            }

            break;
        
        case kROTATE_RIGHT:
            if (last_odometry_msg_counter_ > 0) {
                // ##### Wait on last_ficucial_pose_msg_counter_
                tf::Quaternion qq(last_odometry_msg_.pose.pose.orientation.x,
                                  last_odometry_msg_.pose.pose.orientation.y, 
                                  last_odometry_msg_.pose.pose.orientation.z, 
                                  last_odometry_msg_.pose.pose.orientation.w);
                double current_yaw = tf::getYaw(qq);
                double goal_z = normalizeRadians(start_yaw + goal_z_);
                double goal_z_to_go = normalizeRadians(fabs(current_yaw - goal_z));
                ROS_INFO("[MoveInARectangle::run] kROTATE_RIGHT Goal z: %7.4fr (%7.4fd), current z: %7.4fr still need to rotate  %7.4fr (%7.4fd)",
                         goal_z, 
                         ((goal_z * 360) / (2 * M_PI)), 
                         current_yaw,
                         goal_z_to_go, 
                         ((goal_z_to_go * 360) / (2 * M_PI)));
                
                if (fabs(goal_z_to_go) > 0.04) {
                    ROS_INFO("[MoveInARectangle::run] kROTATE_RIGHT still need to rotate");
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.4;
                    cmd_vel_publisher_.publish(cmd_vel);
                } else {
                    // STOP
                    double odometry_delta = current_yaw - start_yaw;
                    state_ = kDONE;
                    ROS_INFO("[MoveInARectangle::run] kROTATE_RIGHT goal reached, done. Odometry yaw delta: %7.4fr (%7.4fd).",
                             odometry_delta,
                             ((odometry_delta * 360) / (2 * M_PI)));
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_publisher_.publish(cmd_vel);
                    report();
                    return false;
                }
            }

            break;
        
        case KSTART:
            resetEncoders();
            if (last_odometry_msg_counter_ > 0) {
                ROS_INFO("[MoveInARectangle::run] Starting. Change goal to kFORWARD");
                state_ = kFORWARD;
            } else {
                ROS_INFO("[MoveInARectangle::run] Starting. Awaiting odometry info");
            }

            break;

        default:
            ROS_INFO("[MoveInARectangle::run] INVALID STATE: %d", state_);
            break;
    }

    report();
    return true;
}


void MoveInARectangle::resetEncoders() {
    try {
        o_hardware::ResetEncoders values;
        values.request.left = 0;
        values.request.right = 0;
        resetEncodersService_.call(values);
        usleep(250000);
    } catch (...) {
        ROS_ERROR("[MoveInARectangle::resetEncoders] uncaught exception");
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "move_in_a_rectangle");
	ros::NodeHandle nh;

	// MoveInARectangle MoveInARectangle(nh);

	// if (!MoveInARectangle.init()) {
	// 	ROS_ERROR("[motor_controller_node] MoveInARectangle::init failed");
	// 	exit(-1);
	// }

	ROS_INFO("[move_in_a_rectangle] starting loop");
    ros::Rate r(10);
	bool continueGoal = true;
	while (continueGoal && ros::ok()) {
		// continueGoal = MoveInARectangle.run();
        ros::spinOnce();
        r.sleep();
	}

	ROS_INFO("[move_in_a_rectangle] shutdown");
}