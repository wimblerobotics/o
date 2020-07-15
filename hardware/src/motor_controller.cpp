#include "motor_controller.h"
#include "hardware/RoboClawStatus.h"

#include <boost/assign.hpp>
#include <fcntl.h>
#include <iostream>
#include <math.h>
#include <poll.h>
#include <sstream>
#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

//TODO 
// Further initialization?
// No command in n-sec should stop motor.

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg

MotorController::MotorController(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: RoboclawController(nh, urdf_model)
	, motorAlarms_(0)
	, nh_(nh)
	, simulating(false)
	, urdf_model_(urdf_model) {

	assert(ros::param::get("motor_controller/control_loop_hz", controlLoopHz_));
	assert(ros::param::get("motor_controller/m1p", m1p_));
	assert(ros::param::get("motor_controller/m1i", m1i_));
	assert(ros::param::get("motor_controller/m1d", m1d_));
	assert(ros::param::get("motor_controller/m1qpps", m1qpps_));
	assert(ros::param::get("motor_controller/m1p", m2p_));
	assert(ros::param::get("motor_controller/m1i", m2i_));
	assert(ros::param::get("motor_controller/m1d", m2d_));
	assert(ros::param::get("motor_controller/m1qpps", m2qpps_));
	assert(ros::param::get("motor_controller/max_command_retries", maxCommandRetries_));
	assert(ros::param::get("motor_controller/max_m1_current", maxM1Current_));
	assert(ros::param::get("motor_controller/max_m2_current", maxM2Current_));
	assert(ros::param::get("motor_controller/max_seconds_uncommanded_travel", maxSecondsUncommandedTravel_));
	assert(ros::param::get("motor_controller/port_address", portAddress_));
	assert(ros::param::get("motor_controller/quad_pulses_per_meter", quadPulsesPerMeter_));
	assert(ros::param::get("motor_controller/quad_pulses_per_revolution", quadPulsesPerRevolution_));
	assert(ros::param::get("motor_controller/usb_device_name", motorUSBPort_));
	assert(ros::param::get("motor_controller/vmin", vmin_));
	assert(ros::param::get("motor_controller/vtime", vtime_));
	assert(ros::param::get("motor_controller/wheel_radius", wheelRadius_));
	ROS_INFO("[MotorController::MotorController] motor_controller/control_loop_hz: %6.3f", controlLoopHz_);
	ROS_INFO("[MotorController::MotorController] motor_controller/m1p: %6.3f", m1p_);
	ROS_INFO("[MotorController::MotorController] motor_controller/m1i: %6.3f", m1i_);
	ROS_INFO("[MotorController::MotorController] motor_controller/m1d: %6.3f", m1d_);
	ROS_INFO("[MotorController::MotorController] motor_controller/m1qpps: %d", m1qpps_);
	ROS_INFO("[MotorController::MotorController] motor_controller/m2p: %6.3f", m2p_);
	ROS_INFO("[MotorController::MotorController] motor_controller/m2i: %6.3f", m2i_);
	ROS_INFO("[MotorController::MotorController] motor_controller/m2d: %6.3f", m2d_);
	ROS_INFO("[MotorController::MotorController] motor_controller/m2qpps: %d", m2qpps_);
	ROS_INFO("[MotorController::MotorController] motor_controller/max_command_retries: %d", maxCommandRetries_);
	ROS_INFO("[MotorController::MotorController] motor_controller/max_m1_current: %6.3f", maxM1Current_);
	ROS_INFO("[MotorController::MotorController] motor_controller/max_m2_current: %6.3f", maxM2Current_);
	ROS_INFO("[MotorController::MotorController] motor_controller/max_seconds_uncommanded_travel: %6.3f", maxSecondsUncommandedTravel_);
	ROS_INFO_STREAM("[MotorController::MotorController] motor_controller/port_address: 0x" << std::hex << portAddress_);
	ROS_INFO("[MotorController::MotorController] motor_controller/quad_pulses_per_meter: %8.3f", quadPulsesPerMeter_);
	ROS_INFO("[MotorController::MotorController] motor_controller/quad_pulses_per_revolution: %8.3f", quadPulsesPerRevolution_);
	ROS_INFO("[MotorController::MotorController] motor_controller/usb_device_name: %s", motorUSBPort_.c_str());
	ROS_INFO("[MotorController::MotorController] motor_controller/vmin: %d", vmin_);
	ROS_INFO("[MotorController::MotorController] motor_controller/vtime: %d", vtime_);
	ROS_INFO("[MotorController::MotorController] motor_controller/wheel_radius: %6.4f", wheelRadius_);

	now_ = ros::Time::now();
	lastTime_ = now_;

    jointNames_.push_back("front_left_wheel");
    jointNames_.push_back("front_right_wheel");

	// Status
	jointPosition_.resize(jointNames_.size(), 0.0);
	jointVelocity_.resize(jointNames_.size(), 0.0);
	jointEffort_.resize(jointNames_.size(), 0.0);

	// Command
	jointPositionCommand_.resize(jointNames_.size(), 0.0);
	jointVelocityCommand_.resize(jointNames_.size(), 0.0);
	jointEffortCommand_.resize(jointNames_.size(), 0.0);

	// Limits
	jointPositionLowerLimits_.resize(jointNames_.size(), 0.0);
	jointPositionUpperLimits_.resize(jointNames_.size(), 0.0);
	jointVelocityLimits_.resize(jointNames_.size(), 0.0);
	jointEffortLimits_.resize(jointNames_.size(), 0.0);

	// Initialize interfaces for each joint
	for (std::size_t jointId = 0; jointId < jointNames_.size(); ++jointId) {
		ROS_INFO("[MotorController::MotorController] Loading joint name: %s", jointNames_[jointId].c_str());

		// Create joint state interface
		jointStateInterface_.registerHandle
		  (hardware_interface::JointStateHandle(jointNames_[jointId],
						    &jointPosition_[jointId],
						    &jointVelocity_[jointId],
						    &jointEffort_[jointId]));

		// Add command interfaces to joints
		// TODO: decide based on transmissions?
		hardware_interface::JointHandle jointHandlePosition =
		  hardware_interface::JointHandle(jointStateInterface_.getHandle(jointNames_[jointId]),
					      &jointPositionCommand_[jointId]);
		positionJointInterface_.registerHandle(jointHandlePosition);

		hardware_interface::JointHandle jointHandleVelocity =
		  hardware_interface::JointHandle(jointStateInterface_.getHandle(jointNames_[jointId]),
					      &jointVelocityCommand_[jointId]);
		velocityJointInterface_.registerHandle(jointHandleVelocity);

		hardware_interface::JointHandle jointHandleEffort =
		  hardware_interface::JointHandle(jointStateInterface_.getHandle(jointNames_[jointId]),
					      &jointEffortCommand_[jointId]);
		effortJointInterface_.registerHandle(jointHandleEffort);

		// Load the joint limits
		registerJointLimits(jointHandlePosition, jointHandleVelocity, jointHandleEffort, jointId);
	}

	registerInterface(&jointStateInterface_);     // From RobotHW base class.
	registerInterface(&positionJointInterface_);  // From RobotHW base class.
	registerInterface(&velocityJointInterface_);  // From RobotHW base class.
	registerInterface(&effortJointInterface_);    // From RobotHW base class.

	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
	expectedControlLoopDuration_ = ros::Duration(1 / controlLoopHz_);

	statusPublisher_ = nh_.advertise<hardware::RoboClawStatus>("/RoboClawStatus", 1);
	resetEncodersService_ = nh_.advertiseService("reset_encoders", &MotorController::resetEncoders, (MotorController*) this);

	if (!simulating) {
		initHardware();
	}

	ROS_INFO("[MotorController::MotorController] Initialized");
}

void MotorController::initHardware() {
	openPort();	
	setM1PID(m1p_, m1i_, m1d_, m1qpps_);
	setM2PID(m2p_, m2i_, m2d_, m2qpps_);
	hardware::ResetEncoders::Request resetRequest;
	resetRequest.left = 0;
	resetRequest.right = 0;
	hardware::ResetEncoders::Response response;
	resetEncoders(resetRequest, response);
	ROS_INFO("[MotorController::MotorController] RoboClaw software version: %s", getVersion().c_str());
}


MotorController::~MotorController() {

}


void MotorController::controlLoop() {
	ros::Rate rate(controlLoopHz_);
	while(ros::ok()) {
		if (!simulating) {
			update();
		}

		rate.sleep();
	}
}


MotorController::EncodeResult MotorController::getEncoderCommandResult(uint8_t command) {
	uint16_t crc = 0;
	updateCrc(crc, portAddress_);
	updateCrc(crc, command);

	writeN(false, 2, portAddress_, command);
	EncodeResult result = {0, 0};
	uint8_t datum = readByteWithTimeout();
	result.value |= datum << 24;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result.value |= datum << 16;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result.value |= datum << 8;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result.value |= datum;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result.status |= datum;
	updateCrc(crc, datum);

	uint16_t responseCrc = 0;
	datum = readByteWithTimeout();
	responseCrc = datum << 8;
	datum = readByteWithTimeout();
	responseCrc |= datum;
	if (responseCrc == crc) {
		return result;
	}

	ROS_ERROR("[MotorController::getEncoderCommandResult] Expected CRC of: 0x%02X, but got: 0x%02X"
	    	  , int(crc)
	    	  , int(responseCrc));
	throw new TRoboClawException("[MotorController::getEncoderCommandResult] INVALID CRC");
}


uint16_t MotorController::getErrorStatus() {
	for (int retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress_);
			updateCrc(crc, kGETERROR);

			writeN(false, 2, portAddress_, kGETERROR);
			unsigned short result = (unsigned short) getULongCont(crc);
			uint16_t responseCrc = 0;
			uint16_t datum = readByteWithTimeout();
			responseCrc = datum << 8;
			datum = readByteWithTimeout();
			responseCrc |= datum;
			if (responseCrc == crc) {
				return result;
			} else {
				ROS_ERROR("[MotorController::getPIDQ] invalid CRC expected: 0x%02X, got: 0x%02X", crc, responseCrc);
			}
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::getErrorStatus] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::getErrorStatus] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::getErrorStatus] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::getErrorStatus] RETRY COUNT EXCEEDED");
}


std::string MotorController::getErrorString() {
	uint16_t errorStatus = getErrorStatus();
	if (errorStatus == 0) return "normal";
	else {
		std::stringstream errorMessage;
		if (errorStatus & 0x8000) {
			errorMessage << "[M2 Home] ";
		}

		if (errorStatus & 0x4000) {
			errorMessage << "[M1 Home] ";
		}

		if (errorStatus & 0x2000) {
			errorMessage << "[Temperature2 Warning] ";
		}

		if (errorStatus & 0x1000) {
			errorMessage << "[Temperature Warning] ";
		}

		if (errorStatus & 0x800) {
			errorMessage << "[Main Battery Low Warning] ";
		}

		if (errorStatus & 0x400) {
			errorMessage << "[Main Battery High Warning] ";
		}

		if (errorStatus & 0x200) {
			errorMessage << "[M1 Driver Fault] ";
		}

		if (errorStatus & 0x100) {
			errorMessage << "[M2 Driver Fault] ";
		}

		if (errorStatus & 0x80) {
			errorMessage << "[Logic Battery Low Error] ";
		}

		if (errorStatus & 0x40) {
			errorMessage << "[Logic Battery High Error] ";
		}

		if (errorStatus & 0x20) {
			errorMessage << "[Main Battery High Error] ";
		}

		if (errorStatus & 0x10) {
			errorMessage << "Temperature2 Error] ";
		}

		if (errorStatus & 0x08) {
			errorMessage << "[Temperature Error] ";
		}

		if (errorStatus & 0x04) {
			errorMessage << "[E-Stop] ";
		}

		if (errorStatus & 0x02) {
			errorMessage << "[M2 OverCurrent Warning] ";
			motorAlarms_ |= kM2_OVER_CURRENT_ALARM;
		} else {
			motorAlarms_ &= ~kM2_OVER_CURRENT_ALARM;
		}

		if (errorStatus & 0x01) {
			errorMessage << "[M1 OverCurrent Warning] ";
			motorAlarms_ |= kM1_OVER_CURRENT_ALARM;
		} else {
			motorAlarms_ &= ~kM1_OVER_CURRENT_ALARM;
		}

		return errorMessage.str();
	}
}


float MotorController::getLogicBatteryLevel() {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			float result = ((float) get2ByteCommandResult(kGETLBATT)) / 10.0;
			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::getLogicBatteryLevel] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::getLogicBatteryLevel] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::getLogicBatteryLevel] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::getLogicBatteryLevel] RETRY COUNT EXCEEDED");
}


int32_t MotorController::getM1Encoder() {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			EncodeResult result = getEncoderCommandResult(kGETM1ENC);
			return result.value;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::getM1Encoder] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::getM1Encoder] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::getM1Encoder] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::getM1Encoder] RETRY COUNT EXCEEDED");
}


float MotorController::getMainBatteryLevel() {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			float result = ((float) get2ByteCommandResult(kGETMBATT)) / 10.0;
			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND("[MotorController::getMainBatteryLevel] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::getMainBatteryLevel] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::getMainBatteryLevel] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::getMainBatteryLevel] RETRY COUNT EXCEEDED");
}


unsigned short MotorController::get2ByteCommandResult(uint8_t command) {
	uint16_t crc = 0;
	updateCrc(crc, portAddress_);
	updateCrc(crc, command);

	writeN(false, 2, portAddress_, command);
	unsigned short result = 0;
	uint8_t datum = readByteWithTimeout();
	result |= datum << 8;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result |= datum;
	updateCrc(crc, datum);

	uint16_t responseCrc = 0;
	datum = readByteWithTimeout();
	responseCrc = datum << 8;
	datum = readByteWithTimeout();
	responseCrc |= datum;
	if (responseCrc == crc) {
		return result;
	} else {
		ROS_ERROR("[MotorController::get2ByteCommandResult] invalid CRC expected: 0x%02X, got: 0x%02X", crc, responseCrc);
		throw new TRoboClawException("[MotorController::get2ByteCommandResult] INVALID CRC");
	}
}


MotorController::TMotorCurrents MotorController::getMotorCurrents() {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			TMotorCurrents result;
			unsigned long currentPair = getUlongCommandResult(kGETCURRENTS);
			result.m1Current = ((int16_t) (currentPair >> 16)) * 0.010;
			result.m2Current = ((int16_t) (currentPair & 0xFFFF)) * 0.010;
			if (result.m1Current > maxM1Current_) {
				motorAlarms_ |= kM1_OVER_CURRENT_ALARM;
				ROS_ERROR("[MotorController::getMotorCurrents] Moter 1 over current. Max allowed: %6.3f, found: %6.3f",
						  maxM1Current_,
						  result.m1Current);
				stop();
			} else {
				motorAlarms_ &= ~kM1_OVER_CURRENT_ALARM;
			}

			if (result.m2Current > maxM2Current_) {
				motorAlarms_ |= kM2_OVER_CURRENT_ALARM;
				ROS_ERROR("[MotorController::getMotorCurrents] Moter 2 over current. Max allowed: %6.3f, found: %6.3f",
						  maxM2Current_,
						  result.m2Current);
				stop();
			} else {
				motorAlarms_ &= ~kM2_OVER_CURRENT_ALARM;
			}

			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::getMotorCurrents] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::getMotorCurrents] Uncaught exception !!!");
		}
	}

	ROS_ERROR("MotorController::getMotorCurrents] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::getMotorCurrents] RETRY COUNT EXCEEDED");
}


MotorController::TPIDQ MotorController::getPIDQ(uint8_t whichMotor) {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		TPIDQ result;
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress_);
			updateCrc(crc, whichMotor);

			writeN(false, 2, portAddress_, whichMotor);
			result.p = (int32_t) getULongCont(crc);
			result.i = (int32_t) getULongCont(crc);
			result.d = (int32_t) getULongCont(crc);
			result.q = (int32_t) getULongCont(crc);

			uint16_t responseCrc = 0;
			uint16_t datum = readByteWithTimeout();
			responseCrc = datum << 8;
			datum = readByteWithTimeout();
			responseCrc |= datum;
			if (responseCrc == crc) {
				return result;
			} else {
				ROS_ERROR("[MotorController::getPIDQ] invalid CRC expected: 0x%2X, got: 0x%2X", crc, responseCrc);
			}
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::getPIDQ] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::getPIDQ] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::getPIDQ] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::getPIDQ] RETRY COUNT EXCEEDED");
}


float MotorController::getTemperature() {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress_);
			updateCrc(crc, kGETTEMPERATURE);
			writeN(false, 2, portAddress_, kGETTEMPERATURE);
			uint16_t result = 0;
			uint8_t datum = readByteWithTimeout();
			updateCrc(crc, datum);
			result = datum << 8;
			datum = readByteWithTimeout();
			updateCrc(crc, datum);
			result |= datum;

			uint16_t responseCrc = 0;
			datum = readByteWithTimeout();
			responseCrc = datum << 8;
			datum = readByteWithTimeout();
			responseCrc |= datum;
			if (responseCrc == crc) {
				return result / 10.0;
			} else {
				ROS_ERROR("[MotorController::getTemperature] invalid CRC expected: 0x%2X, got: 0x%2X", crc, responseCrc);
			}
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::getTemperature] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::getTemperature] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::getTemperature] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::getTemperature] RETRY COUNT EXCEEDED");
}


unsigned long MotorController::getUlongCommandResult(uint8_t command) {
	uint16_t crc = 0;
	updateCrc(crc, portAddress_);
	updateCrc(crc, command);

	writeN(false, 2, portAddress_, command);
	unsigned long result = 0;
	uint8_t datum = readByteWithTimeout();
	result |= datum << 24;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum << 16;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum << 8;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum;
	updateCrc(crc, datum);

	uint16_t responseCrc = 0;
	datum = readByteWithTimeout();
	responseCrc = datum << 8;
	datum = readByteWithTimeout();
	responseCrc |= datum;
	if (responseCrc == crc) {
		return result;
	}

	ROS_ERROR("[MotorController::getUlongCommandResult] Expected CRC of: 0x%02X, but got: 0x%02X",
	    int(crc),
	    int(responseCrc));
	throw new TRoboClawException("[MotorController::getUlongCommandResult] INVALID CRC");
}


uint32_t MotorController::getULongCont(uint16_t& crc) {
	uint32_t result = 0;
	uint8_t datum = readByteWithTimeout();
	result |= datum << 24;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum << 16;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum << 8;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum;
	updateCrc(crc, datum);
	return result;
}


int32_t MotorController::getVelocity(uint8_t whichVelocity) {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			uint32_t result = getVelocityResult(whichVelocity);
			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::getVelocity] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::getVelocity] Uncaught exception !!!");
		}
	}

	ROS_ERROR("MotorController::getVelocity] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::getVelocity] RETRY COUNT EXCEEDED");
}


int32_t MotorController::getVelocityResult(uint8_t command) {
	uint16_t crc = 0;
	updateCrc(crc, portAddress_);
	updateCrc(crc, command);

	writeN(false, 2, portAddress_, command);
	int32_t result = 0;
	uint8_t datum = readByteWithTimeout();
	result |= datum << 24;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result |= datum << 16;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result |= datum << 8;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result |= datum;
	updateCrc(crc, datum);

	uint8_t direction = readByteWithTimeout();
	updateCrc(crc, direction);
	if (direction != 0) result = -result;

	uint16_t responseCrc = 0;
	datum = readByteWithTimeout();
	responseCrc = datum << 8;
	datum = readByteWithTimeout();
	responseCrc |= datum;
	if (responseCrc == crc) {
		return result;
	}

	ROS_ERROR("[MotorController::getVelocityResult] Expected CRC of: 0x%02X, but got: 0x%02X",
		int(crc),
	    int(responseCrc));
	throw new TRoboClawException("[MotorController::getVelocityResult] INVALID CRC");
}


int32_t MotorController::getM2Encoder() {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			EncodeResult result = getEncoderCommandResult(kGETM2ENC);
			return result.value;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::getM2Encoder] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::getM2Encoder] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::getM2Encoder] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::getM2Encoder] RETRY COUNT EXCEEDED");
}


std::string MotorController::getVersion() {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress_);
			updateCrc(crc, kGETVERSION);
			writeN(false, 2, portAddress_, kGETVERSION);

			uint8_t i;
			uint8_t datum;
			std::stringstream version;

			for (i = 0; i < 48; i++) {
				datum = readByteWithTimeout();
				updateCrc(crc, datum);
				if (datum == 0) {
					uint16_t responseCrc = 0;
					datum = readByteWithTimeout();
					responseCrc = datum << 8;
					datum = readByteWithTimeout();
					responseCrc |= datum;
					if (responseCrc == crc) {
						return version.str();
					} else {
						ROS_ERROR("[MotorController::getVersion] invalid CRC expected: 0x%02X, got: 0x%02X", crc, responseCrc);
					}
				} else {
					version << (char) datum;
				}
			}

			ROS_ERROR("[MotorController::getVersion] unexpected long string");
			throw new TRoboClawException("[MotorController::getVersion] unexpected long string");
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::getVersion] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::getVersion] Uncaught exception !!!");
		}
	}


	ROS_ERROR("[MotorController::getVersion] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::getVersion] RETRY COUNT EXCEEDED");
}


void MotorController::openPort() {
	ROS_INFO("[MotorController::openPort] about to open port: %s", motorUSBPort_.c_str());
	clawPort_ = open(motorUSBPort_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
	if (clawPort_ < 0) {
		ROS_ERROR("[MotorController::openPort] Unable to open USB port: %s, errno: (%d) %s"
				  , motorUSBPort_.c_str()
				  , errno
				  , strerror(errno));
		throw new TRoboClawException("[MotorController::openPort] Unable to open USB port");
	}


    // Fetch the current port settings.
	struct termios portOptions;
	int ret = 0;

 	ret = tcgetattr(clawPort_, &portOptions);
	if (ret < 0) {
		ROS_ERROR("[MotorController::openPort] Unable to get terminal options (tcgetattr), error: %d: %s", errno, strerror(errno));
		// throw new TRoboClawException("[MotorController::openPort] Unable to get terminal options (tcgetattr)");
	}

    // c_cflag contains a few important things- CLOCAL and CREAD, to prevent
    //   this program from "owning" the port and to enable receipt of data.
    //   Also, it holds the settings for number of data bits, parity, stop bits,
    //   and hardware flow control. 
    portOptions.c_cflag |= CLOCAL;            // Prevent changing ownership.
	portOptions.c_cflag |= CREAD;             // Enable reciever.
	portOptions.c_cflag &= ~CRTSCTS;          // Disable hardware CTS/RTS flow control.
	portOptions.c_cflag |= CS8;               // Enable 8 bit characters.
	portOptions.c_cflag &= ~CSIZE;            // Remove size flag.
	portOptions.c_cflag &= ~CSTOPB;           // Disable 2 stop bits.
    portOptions.c_cflag |= HUPCL;             // Enable lower control lines on close - hang up.
	portOptions.c_cflag &= ~PARENB;           // Disable parity.

	//portOptions.c_iflag |= BRKINT;
	portOptions.c_iflag &= ~IGNBRK;            // Disable ignoring break.
	portOptions.c_iflag &= ~IGNCR;             // Disable ignoring CR;
    portOptions.c_iflag &= ~(IGNPAR | PARMRK); // Disable parity checks.
    //portOptions.c_iflag |= IGNPAR;
	portOptions.c_iflag &= ~(INLCR | ICRNL);   // Disable translating NL <-> CR.
	portOptions.c_iflag &= ~INPCK;             // Disable parity checking.
	portOptions.c_iflag &= ~ISTRIP;            // Disable stripping 8th bit.
    portOptions.c_iflag &= ~(IXON | IXOFF);    // disable XON/XOFF flow control

    portOptions.c_lflag &= ~ECHO;			   // Disable echoing characters.
	portOptions.c_lflag &= ~ECHONL;            // ??
    portOptions.c_lflag &= ~ICANON;			   // Disable canonical mode - line by line.
	portOptions.c_lflag &= ~IEXTEN;            // Disable input processing
    portOptions.c_lflag &= ~ISIG;			   // Disable generating signals.
	portOptions.c_lflag &= ~NOFLSH;            // Disable flushing on SIGINT.

	portOptions.c_oflag &= ~OFILL;             // Disable fill characters.
	portOptions.c_oflag &= ~(ONLCR | OCRNL);   // Disable translating NL <-> CR.
	portOptions.c_oflag &= ~OPOST;			   // Disable output processing.
    
    portOptions.c_cc[VKILL] = 8;
    portOptions.c_cc[VMIN] = vmin_;
    portOptions.c_cc[VTIME] = vtime_;
    
    if (cfsetispeed(&portOptions, B38400) < 0) {
		ROS_ERROR("[MotorController::openPort] Unable to set terminal speed (cfsetispeed)");
		throw new TRoboClawException("[MotorController::openPort] Unable to set terminal speed (cfsetispeed)");
    }

    if (cfsetospeed(&portOptions, B38400) < 0) {
		ROS_ERROR("[MotorController::openPort] Unable to set terminal speed (cfsetospeed)");
		throw new TRoboClawException("[MotorController::openPort] Unable to set terminal speed (cfsetospeed)");
    }

    // Now that we've populated our options structure, let's push it back to the system.
    if (tcsetattr(clawPort_, TCSANOW, &portOptions) < 0) {
		ROS_ERROR("[MotorController::openPort] Unable to set terminal options (tcsetattr)");
		throw new TRoboClawException("[MotorController::openPort] Unable to set terminal options (tcsetattr)");
    }
}


void MotorController::publishStatus() {
	hardware::RoboClawStatus roboClawStatus;
    static uint32_t sequenceCount = 0;
	try {
		roboClawStatus.logicBatteryVoltage = getLogicBatteryLevel();
		roboClawStatus.mainBatteryVoltage = getMainBatteryLevel();
		TMotorCurrents motorCurrents = getMotorCurrents();
		roboClawStatus.m1MotorCurrent = motorCurrents.m1Current;
		roboClawStatus.m2MotorCurrent = motorCurrents.m2Current;
		
		TPIDQ pidq = getPIDQ(kGETM1PID);
		roboClawStatus.m1P = pidq.p / 65536.0;
		roboClawStatus.m1I = pidq.i / 65536.0;
		roboClawStatus.m1D = pidq.d / 65536.0;
		roboClawStatus.m1Qpps = pidq.q;
		
		pidq = getPIDQ(kGETM2PID);
		roboClawStatus.m2P = pidq.p / 65536.0;
		roboClawStatus.m2I = pidq.i / 65536.0;
		roboClawStatus.m2D = pidq.d / 65536.0;
		roboClawStatus.m2Qpps = pidq.q;

		roboClawStatus.temperature = getTemperature();

		{
			EncodeResult encoder = getEncoderCommandResult(kGETM1ENC);
			roboClawStatus.encoderM1value = encoder.value;
			roboClawStatus.encoderM1Status = encoder.status;
		}

		{
			EncodeResult encoder = getEncoderCommandResult(kGETM2ENC);
			roboClawStatus.encoderM2value = encoder.value;
			roboClawStatus.encoderM2Status = encoder.status;
		}
		
		roboClawStatus.currentM1Speed = getVelocity(kGETM1SPEED);
		roboClawStatus.currentM2Speed = getVelocity(kGETM2SPEED);

		roboClawStatus.errorString = getErrorString();
		
		statusPublisher_.publish(roboClawStatus);

		if (motorAlarms_ != 0) {
			if (motorAlarms_ & kM1_OVER_CURRENT) {
				ROS_ERROR("[MotorController::publishStatus] M1_OVER_CURRENT");
			}

			if (motorAlarms_ & kM2_OVER_CURRENT) {
				ROS_ERROR("[MotorController::publishStatus] M2_OVER_CURRENT");
			}

			if (motorAlarms_ & kM1_OVER_CURRENT_ALARM) {
				ROS_ERROR("[MotorController::publishStatus] M1_OVER_CURRENT_ALARM");
			}

			if (motorAlarms_ & kM2_OVER_CURRENT_ALARM) {
				ROS_ERROR("[MotorController::publishStatus] M2_OVER_CURRENT_ALARM");
			}
		}
	} catch (TRoboClawException* e) {
		ROS_ERROR("[MotorController::roboClawStatusPublisher] Exception: %s", e->what());
	} catch (...) {
		ROS_ERROR("[MotorController::roboClawStatusPublisher] Uncaught exception !!!");
	}
}


void MotorController::read(const ros::Time& time, const ros::Duration& period) {
	int32_t m1Encoder = getM1Encoder();
	int32_t m2Encoder = getM2Encoder();
	double m1Radians = (m1Encoder / quadPulsesPerRevolution_) * 2.0 * M_PI;
	double m2Radians = (m2Encoder / quadPulsesPerRevolution_) * 2.0 * M_PI;
	
	jointPosition_[0] = m1Radians;
	jointPosition_[1] = m2Radians;
	// ROS_INFO("[MotorController::read] m1:encoder: %d, m2:encoder: %d, m1Radians: %7.4f, m2Radians: %7.4f",
	// 	m1Encoder,
	// 	m2Encoder,
	// 	m1Radians,
	// 	m2Radians);
	publishStatus();
}


uint8_t MotorController::readByteWithTimeout() {
	struct pollfd ufd[1];
	ufd[0].fd = clawPort_;
	ufd[0].events = POLLIN;

	int retval = poll(ufd, 1, 11);
	if (retval < 0) {
		ROS_ERROR("[MotorController::readByteWithTimeout] Poll failed (%d) %s", errno, strerror(errno));
		throw new TRoboClawException("[MotorController::readByteWithTimeout] Read error");
	} else if (retval == 0) {
		std::stringstream ev;
		ev << "[MotorController::readByteWithTimeout] TIMEOUT revents: " << std::hex << ufd[0].revents;
		ROS_ERROR_STREAM(ev.str());
		throw new TRoboClawException("[MotorController::readByteWithTimeout] TIMEOUT");
	} else if (ufd[0].revents & POLLERR) {
		ROS_ERROR("[MotorController::readByteWithTimeout] Error on socket");
		restartPort();
		throw new TRoboClawException("[MotorController::readByteWithTimeout] Error on socket");
	} else if (ufd[0].revents & POLLIN) {
		unsigned char buffer[1];
		ssize_t bytesRead = ::read(clawPort_, buffer, sizeof(buffer));
		if (bytesRead != 1) {
			ROS_ERROR("[MotorController::readByteWithTimeout] Failed to read 1 byte, read: %d", (int) bytesRead);
			throw TRoboClawException("[MotorController::readByteWithTimeout] Failed to read 1 byte");
		}

		static const bool kDEBUG_READBYTE = false;
		if (kDEBUG_READBYTE) {
			if ((buffer[0] < 0x21) || (buffer[0] > 0x7F)) {
				ROS_INFO("..> char: ?? 0x%02X <--", buffer[0]);
			} else {
				ROS_INFO("..> char: %c  0x%02X <--", buffer[0], buffer[0]);
			}
		}

		return buffer[0];
	} else {
		ROS_ERROR("[MotorController::readByteWithTimeout] Unhandled case");
		throw new TRoboClawException("[MotorController::readByteWithTimeout] Unhandled case");
	}
}


bool MotorController::resetEncoders(hardware::ResetEncoders::Request &request,
                       				hardware::ResetEncoders::Response &response) {
	try {
		SetEncoder(kSETM1ENCODER, request.left);
		SetEncoder(kSETM2ENCODER, request.right);
		response.ok = true;
	} catch (...) {
		ROS_ERROR("[MotorController::resetEncoders] uncaught exception");
	}
	return true;
}


void MotorController::restartPort() {
    close(clawPort_);
    usleep(200000);
    openPort();
}

void MotorController::SetEncoder(ROBOCLAW_COMMAND command, long value) {
	int retry;

	if ((command != kSETM1ENCODER) && (command != kSETM2ENCODER)) {
		ROS_ERROR("[MotorController::SetEncoder] Invalid command value");
		throw new TRoboClawException("[MotorController::SetEncoder] Invalid command value");
	}

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			writeN(true, 6, portAddress_, command, 
				   SetDWORDval(value));
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::SetEncoder] Exception: %s, retry number: %d",  e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::SetEncoder] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::SetEncoder] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::SetEncoder] RETRY COUNT EXCEEDED");
}



void MotorController::setM1PID(float p, float i, float d, uint32_t qpps) {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			uint32_t kp = int(p * 65536.0);
			uint32_t ki = int(i * 65536.0);
			uint32_t kd = int(d * 65536.0);
			writeN(true, 18, portAddress_, kSETM1PID, 
				   SetDWORDval(kd),
				   SetDWORDval(kp),
				   SetDWORDval(ki),
				   SetDWORDval(qpps));
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::setM1PID] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::setM1PID] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::setM1PID] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::setM1PID] RETRY COUNT EXCEEDED");
}


void MotorController::setM2PID(float p, float i, float d, uint32_t qpps) {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			uint32_t kp = int(p * 65536.0);
			uint32_t ki = int(i * 65536.0);
			uint32_t kd = int(d * 65536.0);
			writeN(true, 18, portAddress_, kSETM2PID, 
				   SetDWORDval(kd),
				   SetDWORDval(kp),
				   SetDWORDval(ki),
				   SetDWORDval(qpps));
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::setM2PID] Exception: %s, retry number: %d",  e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorController::setM2PID] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::setM2PID] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::setM2PID] RETRY COUNT EXCEEDED");
}


void MotorController::stop() {
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			writeN(true
				   , 6
				   , portAddress_
				   , kMIXEDDUTY
				   , 0, 0, 0, 0);
			ROS_INFO("[MotorController::stop] Stop requested");//#####
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorController::stop] Exception: %s, retry number: %d", e->what(), retry);
			restartPort();
		} catch (...) {
		    ROS_ERROR("[MotorController::stop] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[MotorController::stop] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::stop] RETRY COUNT EXCEEDED");
}


void MotorController::update() {
	now_ = ros::Time::now();
	ros::Duration elapsedDuration = now_ - lastTime_;
	double elapsedTime = (now_ - lastTime_).toSec();
	lastTime_ = now_;
	double controlLoopCycleDurationDeviation = elapsedTime - expectedControlLoopDuration_.toSec();

	if (controlLoopCycleDurationDeviation > (expectedControlLoopDuration_.toSec() / 2.0)) {
		ROS_WARN_STREAM("[MotorController::update] Control loop was too slow by "
						<< controlLoopCycleDurationDeviation
						<< ", actual loop time: "
						<< elapsedTime);
	}

	if (!simulating) {
		read(ros::Time(now_.sec, now_.nsec), elapsedDuration);
		controller_manager_->update(ros::Time(now_.sec, now_.nsec), elapsedDuration);
		write(ros::Time(now_.sec, now_.nsec), elapsedDuration);
	}
}


void MotorController::updateCrc(uint16_t& crc, uint8_t data) {
	crc = crc ^ ((uint16_t) data << 8);
	for (int i = 0; i < 8; i++)	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}
}


void MotorController::write(const ros::Time& time, const ros::Duration& period) {
	// Disabled until better solution found
	// if (motorAlarms_ != 0) {
	// 	ROS_ERROR("[MotorController::write] MOTOR ALARM -- motor stop");
	// 	stop();
	// }

	int retry;
	int32_t leftMaxDistance;
	int32_t rightMaxDistance;

	double leftMetersPerSecond = jointVelocityCommand_[0] * wheelRadius_; // radians/sec * meters/radian.
	double rightMetersPerSecond = jointVelocityCommand_[1] * wheelRadius_; // radians/sec * meters/radian.
	int32_t leftQuadPulsesPerSecond = leftMetersPerSecond * quadPulsesPerMeter_;
	int32_t rightQuadPulsesPerSecond = rightMetersPerSecond * quadPulsesPerMeter_;

	leftMaxDistance = fabs(leftQuadPulsesPerSecond * maxSecondsUncommandedTravel_);
	rightMaxDistance = fabs(rightQuadPulsesPerSecond * maxSecondsUncommandedTravel_);

	if ((fabs(jointVelocityCommand_[0]) > 0.01) ||
		(fabs(jointVelocityCommand_[1]) > 0.01)) {

		for (retry = 0; retry < maxCommandRetries_; retry++) {
			try {
				writeN(true
					   , 19
					   , portAddress_
					   , kMIXEDSPEEDDIST
					   , SetDWORDval(leftQuadPulsesPerSecond)
					   , SetDWORDval(leftMaxDistance)
					   , SetDWORDval(rightQuadPulsesPerSecond)
					   , SetDWORDval(rightMaxDistance)
					   , 1 /* Cancel any previous command */
					   );
				return;
			} catch (TRoboClawException* e) {
				ROS_ERROR("[MotorController::write] Exception: %s, retry number %d", e->what(), retry);
			} catch (...) {
			    ROS_ERROR("[MotorController::write] Uncaught exception !!!");
			}
		}
	} else {
		return;
	}

	ROS_ERROR("[MotorController::write] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorController::write] RETRY COUNT EXCEEDED");
}


void MotorController::writeByte(uint8_t byte) {
	ssize_t result = ::write(clawPort_, &byte, 1);
	//##### ROS_INFO("--> write: 0x%02X", byte); //####
	if (result != 1) {
	  	ROS_ERROR("[MotorController::writeByte] Unable to write one byte, result: %d, errno: %d)", (int) result,  errno);
		restartPort();
		throw new TRoboClawException("[MotorController::writeByte] Unable to write one byte");
	}
}

void MotorController::writeN(bool sendCRC, uint8_t cnt, ...) {
	uint16_t crc = 0;
	va_list marker;
	va_start(marker, cnt);

	int origFlags = fcntl(clawPort_, F_GETFL, 0);
	fcntl(clawPort_, F_SETFL, origFlags & ~O_NONBLOCK);

	for (uint8_t i = 0; i < cnt; i++) {
		uint8_t byte = va_arg(marker, int);
		writeByte(byte);
		updateCrc(crc, byte);
	}

	va_end(marker);

	if (sendCRC) {
		writeByte(crc >> 8);
		writeByte(crc);

		uint8_t response = readByteWithTimeout();
		if (response != 0xFF) {
			ROS_ERROR("[MotorController::writeN] Invalid ACK response");
			throw new TRoboClawException("[MotorController::writeN] Invalid ACK response");
		}
	}

	fcntl(clawPort_, F_SETFL, origFlags);
}


const double MotorController::kBILLION = 1000000000.0;