#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <o_msgs/Monitor.h>
#include <o_msgs/ResetEncoders.h>
#include <o_msgs/RoboClawStatus.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

tf2_ros::Buffer tf_buffer;

ros::Publisher monitorPublisher;            // Publisher of monitor messages.
ros::Subscriber odometry_subscriber;        // Subscriber to nav_msgs/odometry message.
ros::ServiceClient resetEncodersService;    // For reseting the encoders.
ros::Subscriber roboclawStatus_subscriber; 
ros::Subscriber t265_odometry_subscriber;   // Subscriber to nav_msgs/odometry message.

u_long last_odometry_msg_counter = 0;
geometry_msgs::Pose rebased_odometry_pose;
nav_msgs::Odometry last_odometry_msg;

u_long t265_last_odometry_msg_counter = 0;
nav_msgs::Odometry t265_last_odometry_msg;

u_long last_roboClawStatus_msg_counter = 0;
o_msgs::RoboClawStatus last_roboClawStatus_msg;

float motor_current_max = 0.0;
float odom_max_valid_distance_from_t265_odom = 0.0;
double quad_pulses_per_meter = 0.0;

 
//###TODO: When distance > some_delta, capture clock and if too far after some time, correct odom.
//###TODO: Change ResetEncoders to allow x,y,z,w inputs
//###TODO: Also monitor out-of-range motor currents
//###NOTE: Non-stall motor currents about 0-1.22 (typ 0.3) m1, 0-1.2 (typ 0.4) m2
//###NOTE: Stall motor currents 1.8-7.4 (typ 3.4) m1, 1.4-3.3 (typ 2.8) m2

void roboclawStatusCallback(const o_msgs::RoboClawStatus::ConstPtr& msg) {
    last_roboClawStatus_msg = *msg;
    last_roboClawStatus_msg_counter++;
    ROS_INFO("[monitor_node::roboclawStatusCallback] "
             "m1MotorCurrent: %7.4f"
             ", m2MotorCurrent: %7.4f"
             ", motor_current_max: %7.4f"
             , last_roboClawStatus_msg.m1MotorCurrent
             , last_roboClawStatus_msg.m2MotorCurrent
             , motor_current_max
             );
    if ((last_roboClawStatus_msg.m1MotorCurrent > motor_current_max) ||
        (last_roboClawStatus_msg.m2MotorCurrent > motor_current_max)) {
        o_msgs::Monitor alert_message;
        alert_message.message = o_msgs::Monitor::MOTOR_CURRENT_HIGH;
        monitorPublisher.publish(alert_message);
    }
}

void handledomsAreOutOfSync() {
    if ((last_odometry_msg_counter > 0) && (t265_last_odometry_msg_counter > 0)) {
        // We have been receiving the two odometry messages.
        // Compute distance between the two odometry points.
        tf::Transform odom_tf;
        geometry_msgs::Pose o_pose = last_odometry_msg.pose.pose;

        // Make correction to capture difference from last detected wheel sliding event.
        o_pose.position.x += rebased_odometry_pose.position.x;
        o_pose.position.y += rebased_odometry_pose.position.y;

        // Compute vector distance between wheel odometry and visual odometry.
        tf::poseMsgToTF(o_pose, odom_tf);
        tf::Transform t265_odom_tf;
        tf::poseMsgToTF(t265_last_odometry_msg.pose.pose, t265_odom_tf);
        tf::Transform tf = odom_tf.inverseTimes(t265_odom_tf);
        float odoms_distance = 
            sqrt((tf.getOrigin()[0] * tf.getOrigin()[0]) +
                 (tf.getOrigin()[1] * tf.getOrigin()[1]));
        if (odoms_distance > odom_max_valid_distance_from_t265_odom) {
            ROS_INFO("[monitor_node::handledomsAreOutOfSync] orig x: %7.4f"
                    ", y: %7.4f"
                    ", correction x: %7.4f"
                    ", y: %7.4f"
                    ", result x: %7.4f"
                    ", y: %7.4f"
                , last_odometry_msg.pose.pose.position.x
                , last_odometry_msg.pose.pose.position.y
                , rebased_odometry_pose.position.x
                , rebased_odometry_pose.position.y
                , o_pose.position.x
                , o_pose.position.y);

            o_msgs::Monitor alert_message;
            alert_message.message = o_msgs::Monitor::WHEELS_SLIPPING;
            monitorPublisher.publish(alert_message);
            rebased_odometry_pose.position.x = t265_last_odometry_msg.pose.pose.position.x - last_odometry_msg.pose.pose.position.x;
            rebased_odometry_pose.position.y = t265_last_odometry_msg.pose.pose.position.y - last_odometry_msg.pose.pose.position.y;
            ROS_INFO("[handledomsAreOutOfSync] o_x: %f, o_y: %f, o_z: %f, t_x: %f, t_y: %f, t_z: %f", 
                last_odometry_msg.pose.pose.position.x,
                last_odometry_msg.pose.pose.position.y,
                last_odometry_msg.pose.pose.position.z,
                t265_last_odometry_msg.pose.pose.position.x,
                t265_last_odometry_msg.pose.pose.position.y,
                t265_last_odometry_msg.pose.pose.position.z);
            ROS_INFO("[handledomsAreOutOfSync] length %f, 2D-length: %f", 
                tf.getOrigin().length(),
                odoms_distance);
            ROS_ERROR("[monitor_node::handledomsAreOutOfSync]"
                        " odom difference is %7.4f"
                        ", which is greater than the allowed threshold of %7.4f"
                        ", wheel odometry is reset to x: %7.4f, y: %7.4f",
                        odoms_distance,
                        odom_max_valid_distance_from_t265_odom,
                        t265_last_odometry_msg.pose.pose.position.x,
                        t265_last_odometry_msg.pose.pose.position.y);
            ROS_ERROR("rebased x: %7.4f, y: %7.4f", rebased_odometry_pose.position.x, rebased_odometry_pose.position.y);

            if (last_roboClawStatus_msg_counter > 0) {
                    ROS_INFO("[monitor_node]::handledomsAreOutOfSync] m1MotorCurrent: %f, m1MotorCurrent: %f"
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
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "monitor_node");
	ros::NodeHandle nh;

    assert(ros::param::get("monitor/motor_current_max", motor_current_max));
    ROS_INFO("[monitor_node] monitor/motor_current_max: %7.4f", motor_current_max);
    assert(ros::param::get("monitor/odom_max_valid_distance_from_t265_odom", odom_max_valid_distance_from_t265_odom));
	ROS_INFO("[monitor_node] monitor/odom_max_valid_distance_from_t265_odom: %7.4f", odom_max_valid_distance_from_t265_odom);
	assert(ros::param::get("motor_controller/quad_pulses_per_meter", quad_pulses_per_meter));
    ROS_INFO("[monitor_node] motor_controller/quad_pulses_per_meter: %7.4f", quad_pulses_per_meter);

    tf2_ros::TransformListener tf2_listener(tf_buffer);

    odometry_subscriber = nh.subscribe<nav_msgs::Odometry>("diff_drive_controller/odom", 10, odometryCallback);
	monitorPublisher = nh.advertise<o_msgs::Monitor>("monitor", 1);
    resetEncodersService = nh.serviceClient<o_msgs::ResetEncoders>("reset_encoders");
    roboclawStatus_subscriber = nh.subscribe<o_msgs::RoboClawStatus>("RoboClawStatus", 10, roboclawStatusCallback);
    t265_odometry_subscriber = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 10, t265OdometryCallback);

    rebased_odometry_pose.position.x= 0.0;
    rebased_odometry_pose.position.y = 0.0;

	ROS_INFO("[monitor_node] starting loop");
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

	ROS_INFO("[monitor_node] shutdown");
}