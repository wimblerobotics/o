#include "ros/ros.h"

#include "motor_controller.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "motor_controller_node");
	ros::NodeHandle nh;

	ROS_INFO("[motor_controller_node] starting spinner");
	ros::AsyncSpinner spinner(50);
	spinner.start();

  	boost::shared_ptr<MotorController> hw;

	hw.reset(new MotorController(nh));

	ROS_INFO("[motor_controller_node] about to call hw->init");
	hw->init();
	hw->controlLoop();

	return 0;
}