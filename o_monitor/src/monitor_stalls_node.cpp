#include "ros/ros.h"
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <o_hardware/RoboClawStatus.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener tf2_listener(tf_buffer);

ros::Subscriber odometry_subscriber;       // Subscriber to nav_msgs/odometry message.
ros::Subscriber roboclawStatus_subscriber; 
ros::Subscriber t265_odometry_subscriber;  // Subscriber to nav_msgs/odometry message.

u_long last_odometry_msg_counter = 0;
nav_msgs::Odometry last_odometry_msg;

u_long t265_last_odometry_msg_counter = 0;
nav_msgs::Odometry t265_last_odometry_msg;

void roboclawStatusCallback(const o_hardware::RoboClawStatus::ConstPtr& msg) {
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    last_odometry_msg = *msg;
    last_odometry_msg_counter++;
}


void t265OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    t265_last_odometry_msg = *msg;
    try{
        geometry_msgs::PoseStamped t265_pose;

        t265_pose.header = t265_last_odometry_msg.header;
        t265_pose.pose = t265_last_odometry_msg.pose.pose;

        //ROS_INFO(" in: x: %f, y: %f, z: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        geometry_msgs::TransformStamped t265_pose_frame_to_base_link;
        t265_pose_frame_to_base_link = tf_buffer.lookupTransform("base_link", "t265_pose_frame", ros::Time(0), ros::Duration(1.0) );
        tf2::doTransform(t265_pose, t265_pose, t265_pose_frame_to_base_link); 
        ROS_INFO("tf2-out: x: %f, y: %f, z: %f", t265_pose.pose.position.x, t265_pose.pose.position.y, t265_pose.pose.position.z);
        
        t265_last_odometry_msg.pose.pose = t265_pose.pose;
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s" ,ex.what());
    }

    t265_last_odometry_msg_counter++;
    ROS_INFO("[MoveInARectangle::t265OdometryCallback] message received counter:: %lu", last_odometry_msg_counter);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "move_in_a_rectangle");
	ros::NodeHandle nh;

    roboclawStatus_subscriber = nh.subscribe<o_hardware::RoboClawStatus>("RoboClawStatus", 10, roboclawStatusCallback);
    odometry_subscriber = nh.subscribe<nav_msgs::Odometry>("o/diff_drive_controller/odom", 10, odometryCallback);
    t265_odometry_subscriber = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 10, t265OdometryCallback);
	// MoveInARectangle moveInARectangle(nh);

	// if (!moveInARectangle.init()) {
	// 	ROS_ERROR("[move_in_a_rectangle] MoveInARectangle::init failed");
	// 	exit(-1);
	// }

	ROS_INFO("[monitor_stalls_node] starting loop");
    ros::Rate r(10);
	while (ros::ok()) {
        // if (! moveInARectangle.odomMessagesReceived()) {
        //     if (emit_waiting_odometry_message) {
        //         ROS_INFO("[MoveInARectangle] waiting on first odometry message");
        //         emit_waiting_odometry_message = false;
        //     }
        // }

        // if (! moveInARectangle.t265OdomMessagesReceived()) {
        //     if (emit_waiting_t265_odometry_message) {
        //         ROS_INFO("[MoveInARectangle] waiting on first t265 odometry message");
        //         emit_waiting_t265_odometry_message = false;
        //     }
        // }

        // if (moveInARectangle.odomMessagesReceived() && moveInARectangle.t265OdomMessagesReceived()) {
        //     continueGoal = moveInARectangle.run();
        // }

        ros::spinOnce();
        r.sleep();
	}

	ROS_INFO("[monitor_stalls_node] shutdown");
}