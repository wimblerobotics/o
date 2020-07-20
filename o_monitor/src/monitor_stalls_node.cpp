#include "ros/ros.h"
#include <o_hardware/RoboClawStatus.h>


int main(int argc, char** argv) {
	ros::init(argc, argv, "move_in_a_rectangle");
	ros::NodeHandle nh;

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