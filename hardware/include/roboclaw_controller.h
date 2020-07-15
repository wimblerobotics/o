#ifndef __ROBOCLAW_MOTOR_CONTROLLER__
#define __ROBOCLAW_MOTOR_CONTROLLER__

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <transmission_interface/transmission_parser.h>
#include <urdf/model.h>

class RoboclawController : public hardware_interface::RobotHW {
 protected:
	// Startup and shutdown of the internal node inside a roscpp program
	ros::NodeHandle nh_;

	// Model.
	urdf::Model* model_;
	std::vector<std::string> jointNames_;

	// Modes
	bool useRosparamJointLimits_;
	bool useSoftLimitsIfAvailable_;

	// Transmissions
	std::vector<transmission_interface::TransmissionInfo> transmissions_;

	// Hardware interfaces
	hardware_interface::JointStateInterface jointStateInterface_;
	hardware_interface::PositionJointInterface positionJointInterface_;
	hardware_interface::VelocityJointInterface velocityJointInterface_;
	hardware_interface::EffortJointInterface effortJointInterface_;

	// States
	std::vector<double> jointPosition_;
	std::vector<double> jointVelocity_;
	std::vector<double> jointEffort_;

	// Commands
	std::vector<double> jointPositionCommand_;
	std::vector<double> jointVelocityCommand_;
	std::vector<double> jointEffortCommand_;

	// Joint limits interfaces - Saturation
	joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface_;
	joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface_;
	joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface_;

	// Joint limits interfaces - Soft limits
	joint_limits_interface::PositionJointSoftLimitsInterface positionJointSoftLimits_;
	joint_limits_interface::VelocityJointSoftLimitsInterface velocityJointSoftLimits_;
	joint_limits_interface::EffortJointSoftLimitsInterface effortJointSoftLimits_;

	// Copy of limits, in case we need them later in our control stack
	std::vector<double> jointPositionLowerLimits_;
	std::vector<double> jointPositionUpperLimits_;
	std::vector<double> jointVelocityLimits_;
	std::vector<double> jointEffortLimits_;

 public:
	RoboclawController(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL) ;
	~RoboclawController();

	/**
	* Get the transmissions in the model.
	*/
	const std::vector<transmission_interface::TransmissionInfo>& getTransmissions() {
		return transmissions_;
	}

	/**
	* Initialize the robot hardware interface.
	*/
	virtual void init();

	// Has the model been loaded?
	virtual bool modelLoaded();

	/**
	  * Reads data from the robot HW
	  *
	  * \param time The current time
	  * \param period The time passed since the last call to \ref read
	  */
	virtual void read(const ros::Time& time, const ros::Duration& period);

	void reset();

	/**
 	 * Writes data to the robot HW
	 *
	 * \param time The current time
	 * \param period The time passed since the last call to \ref write
	 */
	virtual void write(const ros::Time& time, const ros::Duration& period);

 protected:
	void loadURDF(ros::NodeHandle &nh, std::string param_name);

	/**
	   * \brief Register the limits of the joint specified by joint_id and joint_handle. The limits
	   * are retrieved from the urdf_model.
	   *
	   * \return the joint's type, lower position limit, upper position limit, and effort limit.
	   */
  	void registerJointLimits(const hardware_interface::JointHandle &joint_handle_position,
                             const hardware_interface::JointHandle &joint_handle_velocity,
                             const hardware_interface::JointHandle &joint_handle_effort,
                             std::size_t joint_id);};

#endif
