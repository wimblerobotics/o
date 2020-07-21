#ifndef __MOTOR_CONTROLLER__
#define __MOTOR_CONTROLLER__

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include "o_hardware/motor_controllerConfig.h"
#include "o_msgs/ResetEncoders.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "roboclaw_controller.h"

class MotorController : public RoboclawController {
public:
	struct TRoboClawException : public std::exception {
		std::string s;
		TRoboClawException(std::string ss) : s(ss) {}
		~TRoboClawException() throw() {}
		const char* what() const throw() { return s.c_str(); }
	};

	MotorController(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

	~MotorController();

	void controlLoop();

	virtual void read(const ros::Time& time, const ros::Duration& period);

	virtual void update();

	virtual void write(const ros::Time& time, const ros::Duration& period);

	// Get RoboClaw error status bits.
	uint16_t getErrorStatus();

	// Get RoboClaw error status as a string.
	std::string getErrorString();

	float getLogicBatteryLevel();

	float getMainBatteryLevel();

	typedef struct {
		int32_t		value;
		uint8_t		status;
	} EncodeResult;

	// Get the encoder value for motor 1.
	int32_t getM1Encoder();

	// Get the encoder value for motor 2.
	int32_t getM2Encoder();

	typedef struct {
		float m1Current;
		float m2Current;
	} TMotorCurrents;

	TMotorCurrents getMotorCurrents();

	typedef struct {
		int32_t p;
		int32_t i;
		int32_t d;
		int32_t q;
	} TPIDQ;

	TPIDQ getPIDQ(uint8_t whichMotor);
	
	int32_t getM1Speed();

	TPIDQ getM2PIDQ();

	int32_t getM2Speed();

	// Get RoboClaw software versions.
	std::string getVersion();

	// Stop motion.
	void stop();

private:
	// For dynamic reconfiguration
	 dynamic_reconfigure::Server<motor_controller::motor_controllerConfig> dynamicConfigurationServer_;
	 dynamic_reconfigure::Server<motor_controller::motor_controllerConfig>::CallbackType dynamicConfigurationCallback_;
	// For publishing the RoboClawStatus;
	ros::Publisher statusPublisher_;

	typedef struct {
		unsigned long p1;
		unsigned long p2;
	} ULongPair;

	enum {
		kERROR_NORMAL = 0x00,
		kM1OVERCURRENT = 0x01,
		kM2OVERCURRENT = 0x02,
		kESTOP = 0x04,
		kTEMPERATURE = 0x08,
		kMAINBATTERYHIGH = 0x10,
		kMAINBATTERYLOW = 0x20,
		kLOGICBATTERYHIGH = 0x40,
		kLOGICBATTERYLOW = 0x80
	};

	// Enum values without a 'k' prefix have not been used in code.
	typedef enum ROBOCLAW_COMMAND{
		M1FORWARD = 0,
	    M1BACKWARD = 1,
	    SETMINMB = 2,
	    SETMAXMB = 3,
	    M2FORWARD = 4,
	    M2BACKWARD = 5,
	    M17BIT = 6,
	    M27BIT = 7,
	    MIXEDFORWARD = 8,
	    MIXEDBACKWARD = 9,
	    MIXEDRIGHT = 10,
	    MIXEDLEFT = 11,
	    MIXEDFB = 12,
	    MIXEDLR = 13,
	    kGETM1ENC = 16,
	    kGETM2ENC = 17,
	    kGETM1SPEED = 18,
	    kGETM2SPEED = 19,
	    RESETENC = 20,
	    kGETVERSION = 21,
		kSETM1ENCODER = 22,
		kSETM2ENCODER = 23,
	    kGETMBATT = 24,
	    kGETLBATT = 25,
	    SETMINLB = 26,
	    SETMAXLB = 27,
	    kSETM1PID = 28,
	    kSETM2PID = 29,
	    GETM1ISPEED = 30,
	    GETM2ISPEED = 31,
	    M1DUTY = 32,
	    M2DUTY = 33,
	    kMIXEDDUTY = 34,
	    M1SPEED = 35,
	    M2SPEED = 36,
	    kMIXEDSPEED = 37,
	    M1SPEEDACCEL = 38,
	    M2SPEEDACCEL = 39,
	    MIXEDSPEEDACCEL = 40,
	    M1SPEEDDIST = 41,
	    M2SPEEDDIST = 42,
	    kMIXEDSPEEDDIST = 43,
	    M1SPEEDACCELDIST = 44,
	    M2SPEEDACCELDIST = 45,
	    MIXEDSPEEDACCELDIST = 46,
	    GETBUFFERS = 47,
	    SETPWM = 48,
	    kGETCURRENTS = 49,
	    MIXEDSPEED2ACCEL = 50,
	    MIXEDSPEED2ACCELDIST = 51,
	    M1DUTYACCEL = 52,
	    M2DUTYACCEL = 53,
	    MIXEDDUTYACCEL = 54,
	    kGETM1PID = 55,
	    kGETM2PID = 56,
	    kGETTEMPERATURE = 82,
	    kGETERROR = 90,
	    WRITENVM = 94,
		GETM1MAXCURRENT = 135
		} ROBOCLAW_COMMAND;

	int clawPort_;							// Unix file descriptor for RoboClaw connection.
	double controlLoopHz_;					// Loop rate for control loop.
	bool resetControlLoopHz_;				// True => value was changed via dynamic configuration
	float m1p_;
	float m1i_;
	float m1d_;
	int m1qpps_;
	float m2p_;
	float m2i_;
	float m2d_;
	int m2qpps_;
	int maxCommandRetries_;					// Maximum number of times to retry a RoboClaw command.
	float maxM1Current_;					// Maximum allowed M1 current.
	float maxM2Current_;					// Maximum allowed M2 current.
	double maxSecondsUncommandedTravel_;	// Abort travel after this number of seconds if a new motion command has not arrived.
	int motorAlarms_;						// Motors alarms. Bit-wise or of contributors.
	std::string motorUSBPort_;				// Device name of RoboClaw device.
	int portAddress_;						// Port number of RoboClaw device under control
	double quadPulsesPerMeter_;				// Number of quadrature pulses that will be received after 1 meter of travel.
	double quadPulsesPerRevolution_;		// Number of quadrature pulses per revolution.
	bool simulating;						// True => robot is runnig in sumulator.
	int vmin_;								// Terminal control value.
	int vtime_;								// Terminal control value.
	double wheelRadius_;					// Wheel radius.

    motor_controller::motor_controllerConfig controllerConfiguration_;

	enum {
		kM1_OVER_CURRENT = 0x01,			// Moter 1 current sense is too high.
		kM2_OVER_CURRENT = 0x02,			// Motor 2 current sense is too high.
		kM1_OVER_CURRENT_ALARM = 0x04,		// Motor 1 controller over current alarm.
		kM2_OVER_CURRENT_ALARM = 0x08,		// Motor 2 controller over current alarm.
	};

	static const double kBILLION;

	ros::NodeHandle nh_;
	ros::ServiceServer resetEncodersService_;

	urdf::Model *urdf_model_;

	// Time.
	ros::Duration expectedControlLoopDuration_;
	ros::Time lastTime_;
	ros::Time now_;

	

	/** \brief ROS Controller Manager and Runner
	 *
	 * This class advertises a ROS interface for loading, unloading, starting, and
	 * stopping ros_control-based controllers. It also serializes execution of all
	 * running controllers in \ref update.
	 */
	boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    // Process dynamic configuration callbacks.
    void dynamicConfigurationCallback(motor_controller::motor_controllerConfig &config, uint32_t level);


	// Get the encoder result given a command which indicates which motor to interrogate.
	EncodeResult getEncoderCommandResult(uint8_t command);

	// Get velocity (speed) of a motor.
	int32_t getVelocity(uint8_t whichVelocity);

	// Get velocity (speed) result from the RoboClaw controller.
	int32_t getVelocityResult(uint8_t command);

	float getTemperature();

	unsigned long getUlongCommandResult(uint8_t command);

	uint32_t getULongCont(uint16_t& crc);

	unsigned short get2ByteCommandResult(uint8_t command);

	// Initialize the o_hardware.
	void initHardware();

	// Open the RoboClaw USB port.
	void  openPort();

	// Read one byte from device with timeout.
	uint8_t readByteWithTimeout();

	// Perform error recovery to re-open a failed device port.
	void restartPort();

	// Publish the RoboClaw status.
	void publishStatus();

	// Reset the encoders.
	bool resetEncoders(o_msgs::ResetEncoders::Request &request,
                       o_msgs::ResetEncoders::Response &response);
					   
	// Set the PID for motor M1.
	void setM1PID(float p, float i, float d, uint32_t qpps);

	// Set the PID for motor M1.
	void setM2PID(float p, float i, float d, uint32_t qpps);

	// Update the running CRC result.
	void updateCrc(uint16_t& crc, uint8_t data);

	// Write one byte to the device.
	void writeByte(uint8_t byte);

	// Write a stream of bytes to the device.
	void writeN(bool sendCRC, uint8_t cnt, ...);

	void SetEncoder(ROBOCLAW_COMMAND command, long value);
};

#endif
