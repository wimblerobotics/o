#include "ros/ros.h"
#include "move_in_a_rectangle.h"

#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <iomanip>      // std::setprecision
#include <iostream>
#include <o_msgs/ResetEncoders.h>
#include <ros/console.h>
#include <string>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <unistd.h>
#include <vector>

u_long MoveInARectangle::last_odometry_msg_counter_ = 0;
u_long MoveInARectangle::t265_last_odometry_msg_counter_ = 0;

MoveInARectangle::MoveInARectangle(ros::NodeHandle& nh)
	: step_x_(1.0)
    ,  step_z_(M_PI)
    , nh_(nh)
    , prev_(ros::Time::now())
    , start_odometry_found_(false)
    , t265_start_odometry_found_(false)
    , state_(KSTART)
    , tf2_listener_(tf_buffer_) {
    resetEncodersService_ = nh.serviceClient<o_msgs::ResetEncoders>("reset_encoders");
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
    //tf::StampedTransform transform;
    try{
        geometry_msgs::PoseStamped t265_pose;

        t265_pose.header = t265_last_odometry_msg_.header;
        t265_pose.pose = t265_last_odometry_msg_.pose.pose;

        //ROS_INFO(" in: x: %f, y: %f, z: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        geometry_msgs::TransformStamped t265_pose_frame_to_base_link;
        t265_pose_frame_to_base_link = tf_buffer_.lookupTransform("base_link", "t265_pose_frame", ros::Time(0), ros::Duration(1.0) );
        tf2::doTransform(t265_pose, t265_pose, t265_pose_frame_to_base_link); 
        //ROS_INFO("tf2-out: x: %f, y: %f, z: %f", t265_pose.pose.position.x, t265_pose.pose.position.y, t265_pose.pose.position.z);
        
        t265_last_odometry_msg_.pose.pose = t265_pose.pose;
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s" ,ex.what());
    }

    t265_last_odometry_msg_counter_++;
    if (!t265_start_odometry_found_) {
        t265_start_odometry_ = *msg;
        t265_start_odometry_found_ = true;
    }
    //ROS_INFO("[MoveInARectangle::t265OdometryCallback] message received counter:: %lu", t265_last_odometry_msg_counter);
}


std::string MoveInARectangle::eulerString(const geometry_msgs::Quaternion& q, u_long counter) {
    tf::Quaternion qq(q.x, q.y, q.z, q.w);
    std::stringstream s;

    s.flags(std::ios::fixed);
    s << "angle: " << std::setprecision(4);
    if (counter > 0) {
        double yaw = tf::getYaw(qq);
        s << yaw;
        s << "r (" << std::setw(10) << ((yaw * 360) / (2 * M_PI)) << "d)";
        s << std::setprecision(4);
        s << ", QUAT x: " << std::setw(10) << q.x;
        s << ", y: " << std::setw(10) << q.y;
        s << ", z: " << std::setw(10) << q.z;
        s << ", w: " << std::setw(10) << q.w;
    } else {
        s << "NO DATA";
    }
    
    return s.str();
}


void MoveInARectangle::report() {
    std::stringstream s;
    std::string calibStatus = "??";

    s.flags(std::ios::fixed);
    s << std::endl;
    s << "     ODOM " << std::setprecision(2) << eulerString(last_odometry_msg_.pose.pose.orientation, last_odometry_msg_counter_);
    if (last_odometry_msg_counter_ > 0) {
        s << ",       ODOM pos x: " << std::setprecision(4) 
        << std::setw(10) << last_odometry_msg_.pose.pose.position.x 
        << ", y: " 
        << std::setw(10) << last_odometry_msg_.pose.pose.position.y 
        << ", z: "
        << std::setw(10) << last_odometry_msg_.pose.pose.position.z;
    } else {
        s << " ODOM NO DATA";
    }

    s << std::endl;

    s << "t265 ODOM " << std::setprecision(2) << eulerString(t265_last_odometry_msg_.pose.pose.orientation, t265_last_odometry_msg_counter_);
    if (t265_last_odometry_msg_counter_ > 0) {
        s << ",  t265 ODOM pos x: " << std::setprecision(4) 
        << std::setw(10) << t265_last_odometry_msg_.pose.pose.position.x 
        << ", y: " 
       << std::setw(10)  << t265_last_odometry_msg_.pose.pose.position.y 
        << ", z: "
        << std::setw(10) << t265_last_odometry_msg_.pose.pose.position.z;
    } else {
        s << ", t265  ODOM NO DATA";
    }

    s  << std::endl;

    ROS_INFO_STREAM("[MoveInARectangle::report] "  << s.str());
}


std::string MoveInARectangle::getStateString(STATE state) {
    std::string state_string = "????";
    double start_yaw = 0.0;
    switch (state_) {
        case kDONE: state_string = "DONE"; break;
        case kFORWARD: state_string = "kFORWARD"; break;
        case kROTATE_LEFT: state_string = "kROTATE_LEFT"; break;
        case KSTART: state_string = "KSTART"; break;
    }

    return state_string;
}

std::string MoveInARectangle::reportOdom(bool odometry_found, nav_msgs::Odometry odometry, std::string prefix) {
    std::stringstream result;
    double start_yaw = 0.0;

    result << prefix;
    if (odometry_found > 0) {
        tf::Quaternion qq(odometry.pose.pose.orientation.x,
                          odometry.pose.pose.orientation.y, 
                          odometry.pose.pose.orientation.z, 
                          odometry.pose.pose.orientation.w);
        start_yaw = tf::getYaw(qq);
        result << std::setprecision(4);
        result << "start x: " << odometry.pose.pose.position.x;
        result << ", y: " << odometry.pose.pose.position.y;
        result << ", z: " << odometry.pose.pose.position.z;
        result << ", start_yaw: " << start_yaw << "r (" << ((start_yaw * 360) / (2 * M_PI)) << "d)";
    } else {
        result << "NO ODOM";
    }

    return result.str();
}



bool MoveInARectangle::run() {
    std::stringstream start_goal_string;
     double start_yaw = 0.0;

     if (start_odometry_found_ > 0) {
        tf::Quaternion qq(start_odometry_.pose.pose.orientation.x,
                          start_odometry_.pose.pose.orientation.y, 
                          start_odometry_.pose.pose.orientation.z, 
                          start_odometry_.pose.pose.orientation.w);
        start_yaw = tf::getYaw(qq);
     }

    start_goal_string << reportOdom(start_odometry_found_, start_odometry_, "");
    start_goal_string << reportOdom(t265_start_odometry_found_, t265_start_odometry_, ", t265_");
    ROS_INFO_STREAM("[MoveInARectangle::run] state: "
                    << getStateString(state_)
                    << ", "
                    << start_goal_string.str());

    switch (state_) {
        case kDONE:
            return false;
            break;
        
        case kFORWARD:
            if (last_odometry_msg_counter_ > 0) {
                double goal_x = start_odometry_.pose.pose.position.x + step_x_;
                double goal_x_to_go =  goal_x - last_odometry_msg_.pose.pose.position.x;
                ROS_INFO("[MoveInARectangle::run] kFORWARD goal x: %7.4f, x remaining to go %7.4f m", 
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
                    state_ = kROTATE_LEFT;
                    ROS_INFO("[MoveInARectangle::run] kFORWARD goal reached, begin rotation. Odometry distance moved: %7.4f.", odometry_x_moved);
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    cmd_vel_publisher_.publish(cmd_vel);
                    resetEncoders();
                }
            }

            break;
        
        case kROTATE_LEFT:
            if (last_odometry_msg_counter_ > 0) {
                tf::Quaternion qq(last_odometry_msg_.pose.pose.orientation.x,
                                  last_odometry_msg_.pose.pose.orientation.y, 
                                  last_odometry_msg_.pose.pose.orientation.z, 
                                  last_odometry_msg_.pose.pose.orientation.w);
                double current_yaw = tf::getYaw(qq);
                double goal_z = normalizeRadians(start_yaw + step_z_);
                double goal_z_to_go = normalizeRadians(fabs(current_yaw - goal_z));
                ROS_INFO("[MoveInARectangle::run] kROTATE_LEFT Goal z: %7.4fr (%7.4fd), current z: %7.4fr still need to rotate  %7.4fr (%7.4fd)",
                         goal_z, 
                         ((goal_z * 360) / (2 * M_PI)), 
                         current_yaw,
                         goal_z_to_go, 
                         ((goal_z_to_go * 360) / (2 * M_PI)));
                
                if (fabs(goal_z_to_go) > 0.04) {
                    ROS_INFO("[MoveInARectangle::run] kROTATE_LEFT still need to rotate");
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.4;
                    cmd_vel_publisher_.publish(cmd_vel);
                } else {
                    // STOP
                    double odometry_delta = current_yaw - start_yaw;
                    state_ = kDONE;
                    ROS_INFO("[MoveInARectangle::run] kROTATE_LEFT goal reached, done. Odometry yaw delta: %7.4fr (%7.4fd).",
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
            if ((last_odometry_msg_counter_ > 0) && (t265_last_odometry_msg_counter_ > 0)) {
                ROS_INFO("[MoveInARectangle::run] Starting. Change goal to kFORWARD");
                state_ = kFORWARD;
            } else {
                ROS_INFO("[MoveInARectangle::run] Starting. Awaiting odometry info");
            }

            break;

        default:
            ROS_INFO("[MoveInARectangle::run] INVALID STATE: %d", state_);
            report();
            return false;
            break;
    }

    report();
    return true;
}


void MoveInARectangle::resetEncoders() {
    try {
        o_msgs::ResetEncoders values;
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

	MoveInARectangle moveInARectangle(nh);

	if (!moveInARectangle.init()) {
		ROS_ERROR("[move_in_a_rectangle] MoveInARectangle::init failed");
		exit(-1);
	}

	ROS_INFO("[move_in_a_rectangle] starting loop");
    ros::Rate r(10);
	bool continueGoal = true;
    bool emit_waiting_odometry_message = true;
    bool emit_waiting_t265_odometry_message = true;
	while (continueGoal && ros::ok()) {
        if (! moveInARectangle.odomMessagesReceived()) {
            if (emit_waiting_odometry_message) {
                ROS_INFO("[MoveInARectangle] waiting on first odometry message");
                emit_waiting_odometry_message = false;
            }
        }

        if (! moveInARectangle.t265OdomMessagesReceived()) {
            if (emit_waiting_t265_odometry_message) {
                ROS_INFO("[MoveInARectangle] waiting on first t265 odometry message");
                emit_waiting_t265_odometry_message = false;
            }
        }

        if (moveInARectangle.odomMessagesReceived() && moveInARectangle.t265OdomMessagesReceived()) {
            continueGoal = moveInARectangle.run();
        }

        ros::spinOnce();
        r.sleep();
	}

	ROS_INFO("[move_in_a_rectangle] shutdown");
}