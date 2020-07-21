#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <o_msgs/ResetEncoders.h>
#include <o_msgs/RoboClawStatus.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

tf2_ros::Buffer tf_buffer;
ros::ServiceClient resetEncodersService;   // For reseting the encoders.


ros::Subscriber odometry_subscriber;       // Subscriber to nav_msgs/odometry message.
ros::Subscriber roboclawStatus_subscriber; 
ros::Subscriber t265_odometry_subscriber;  // Subscriber to nav_msgs/odometry message.

u_long last_odometry_msg_counter = 0;
nav_msgs::Odometry last_odometry_msg;

u_long t265_last_odometry_msg_counter = 0;
nav_msgs::Odometry t265_last_odometry_msg;

u_long last_roboClawStatus_msg_counter = 0;
o_msgs::RoboClawStatus last_roboClawStatus_msg;

float odom_max_valid_distance_from_t265_odom = 0.0;

 
//###TODO: Also sense on t265_last_odometry_msg change.
//###TODO: When distance > some_delta, capture clock and if too far after some time, correct odom.
//###TODO: Change ResetEncoders to allow x,y,z,w inputs
//###TODO: Also monitor out-of-range motor currents
//###NOTE: Non-stall motor currents about 0-1.22 (typ 0.3) m1, 0-1.2 (typ 0.4) m2
//###NOTE: Stall motor currents 1.8-7.4 (typ 3.4) m1, 1.4-3.3 (typ 2.8) m2

void roboclawStatusCallback(const o_msgs::RoboClawStatus::ConstPtr& msg) {
    last_roboClawStatus_msg = *msg;
    last_roboClawStatus_msg_counter++;
}

void handledomsAreOutOfSync() {
    if ((last_odometry_msg_counter > 0) && (t265_last_odometry_msg_counter > 0)) {
        // We have been receiving the two odometry messages.
        // Compute distance between the two odometry points.
        tf::Transform odom_tf;
        tf::poseMsgToTF(last_odometry_msg.pose.pose, odom_tf);
        tf::Transform t265_odom_tf;
        tf::poseMsgToTF(t265_last_odometry_msg.pose.pose, t265_odom_tf);
        tf::Transform tf = odom_tf.inverseTimes(t265_odom_tf);
        float odoms_distance = 
            sqrt((tf.getOrigin()[0] * tf.getOrigin()[0]) +
                 (tf.getOrigin()[1] * tf.getOrigin()[1]));
        // ROS_INFO("[odometryCallback] o_x: %f, o_y: %f, o_z: %f, t_x: %f, t_y: %f, t_z: %f", 
        //     last_odometry_msg.pose.pose.position.x,
        //     last_odometry_msg.pose.pose.position.y,
        //     last_odometry_msg.pose.pose.position.z,
        //     t265_last_odometry_msg.pose.pose.position.x,
        //     t265_last_odometry_msg.pose.pose.position.y,
        //     t265_last_odometry_msg.pose.pose.position.z);
        // ROS_INFO("[odometryCallback] length %f, 2D-length: %f", 
        //     tf.getOrigin().length(),
        //     odoms_distance)p
        // );

        if (odoms_distance > odom_max_valid_distance_from_t265_odom) {
            try {
                ROS_ERROR("[monitor_stalls.handledomsAreOutOfSync]"
                          " odom difference is %7.4f"
                          ", which is greater than the allowed threshold of %7.4f"
                          ", wheel odometry is reset to x: %7.4f, y: %7.4f",
                          odoms_distance,
                          odom_max_valid_distance_from_t265_odom,
                          t265_last_odometry_msg.pose.pose.position.x,
                          t265_last_odometry_msg.pose.pose.position.y);
                o_msgs::ResetEncoders values;
                values.request.left = t265_last_odometry_msg.pose.pose.position.x;
                values.request.right = t265_last_odometry_msg.pose.pose.position.y;
                resetEncodersService.call(values);
                usleep(250000);
            } catch (...) {
                ROS_ERROR("[monitor_stalls.handledomsAreOutOfSync] uncaught exception");
            }

            if (last_roboClawStatus_msg_counter > 0) {
                    ROS_INFO("[monitor_stalls.handledomsAreOutOfSync] m1MotorCurrent: %f, m1MotorCurrent: %f"
                            , last_roboClawStatus_msg.m1MotorCurrent
                            , last_roboClawStatus_msg.m2MotorCurrent);
            }
        }
    }
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {    
    last_odometry_msg = *msg;
    last_odometry_msg_counter++;
    handledomsAreOutOfSync();
}

void t265OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    t265_last_odometry_msg = *msg;
    try{
        // Translate the t265 odom values to base_link values to match wheel odometry.
        geometry_msgs::PoseStamped t265_pose;

        t265_pose.header = t265_last_odometry_msg.header;
        t265_pose.pose = t265_last_odometry_msg.pose.pose;

        //ROS_INFO(" in: x: %f, y: %f, z: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

        geometry_msgs::TransformStamped t265_pose_frame_to_base_link;
        t265_pose_frame_to_base_link = tf_buffer.lookupTransform("base_link", "t265_pose_frame", ros::Time(0), ros::Duration(1.0) );
        tf2::doTransform(t265_pose, t265_pose, t265_pose_frame_to_base_link); 
        // ROS_INFO("tf2-out: x: %f, y: %f, z: %f", t265_pose.pose.position.x, t265_pose.pose.position.y, t265_pose.pose.position.z);
        
        t265_last_odometry_msg.pose.pose = t265_pose.pose;
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s" ,ex.what());
    }

    t265_last_odometry_msg_counter++;
    handledomsAreOutOfSync();

    // ROS_INFO("[t265OdometryCallback] message received counter:: %lu", t265_last_odometry_msg_counter);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "monitor_stalls_node");
	ros::NodeHandle nh;

    assert(ros::param::get("monitor/odom_max_valid_distance_from_t265_odom", odom_max_valid_distance_from_t265_odom));
	ROS_INFO("[monitor_stalls_node] onitor/odom_max_valid_distance_from_t265_odom: %6.3f", odom_max_valid_distance_from_t265_odom);
	

    tf2_ros::TransformListener tf2_listener(tf_buffer);

    roboclawStatus_subscriber = nh.subscribe<o_msgs::RoboClawStatus>("RoboClawStatus", 10, roboclawStatusCallback);
    odometry_subscriber = nh.subscribe<nav_msgs::Odometry>("o/diff_drive_controller/odom", 10, odometryCallback);
    t265_odometry_subscriber = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 10, t265OdometryCallback);
    resetEncodersService = nh.serviceClient<o_msgs::ResetEncoders>("reset_encoders");

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