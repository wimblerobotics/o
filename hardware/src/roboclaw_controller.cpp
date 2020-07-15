#include "ros/ros.h"

#include "motor_controller.h"
#include <regex>

RoboclawController::RoboclawController(ros::NodeHandle &nh, urdf::Model *urdf_model)
    : model_(NULL)
    , nh_(nh)
    , useRosparamJointLimits_(false)
    , useSoftLimitsIfAvailable_(false) {
    if (urdf_model == NULL) {
        ROS_INFO_STREAM("[RoboclawController::RoboclawController] NO urdf model is given, about to load");
        loadURDF(nh, "robot_description");
    } else {
        ROS_INFO_STREAM("[RoboclawController::RoboclawController] urdf model is given");
        model_ = urdf_model;
    }
}

RoboclawController::~RoboclawController() {
}


void RoboclawController::init() {
}


void RoboclawController::loadURDF(ros::NodeHandle &nh, std::string param_name) {
    std::string urdf_string;
    model_ = new urdf::Model();
    ROS_INFO_STREAM("[RoboclawController::loadURDF] param_name:"
                    << param_name);
    // search and wait for robot_description on param server
    while (urdf_string.empty()/*### && ros::ok()###*/) {
        std::string search_param_name;
        if (nh.searchParam(param_name, search_param_name)) {
            ROS_INFO_STREAM("[RoboclawController::loadURDF] In namespace: "
                            << nh.getNamespace()
                            << ", waiting for model URDF"
                            " on the ROS param server at search_param_name: "
                            << search_param_name);
            nh.getParam(search_param_name, urdf_string);
            break;
        } else {
            ROS_INFO_STREAM("[RoboclawController::loadURDF] In namespace: "
                            << nh.getNamespace()
                            << ", waiting for model URDF"
                            " on the ROS param server at param_name: "
                            << param_name);
            nh.getParam(param_name, urdf_string);
            break;
        }

        usleep(10000);
    }

    ROS_INFO_STREAM("[RoboclawController::loadURDF] fetched: " << urdf_string.length() << " bytes");
    ROS_INFO_STREAM("[RoboclawController::loadURDF] about to initialize model with string");
    if (!model_->initString(urdf_string)) {
        ROS_ERROR_STREAM("[RoboclawController::loadURDF] Unable to initialize model with loaded URDF model");
    } else {
        ROS_INFO_STREAM("[RoboclawController::loadURDF] URDF model successfully initialized");
    }

    transmission_interface::TransmissionParser parser;

    bool transmissions_parsed = parser.parse(urdf_string, transmissions_);
    if (!transmissions_parsed) {
        ROS_ERROR_STREAM("[RoboclawController::loadURDF] could not parse URDF for transmissions");
    }

    ROS_INFO_STREAM("[RoboclawController::loadURDF] transmissions found: " << transmissions_.size());

}

bool RoboclawController::modelLoaded() {
    return model_ != NULL;
}

void RoboclawController::registerJointLimits(const hardware_interface::JointHandle &jointHandlePosition,
        const hardware_interface::JointHandle &jointHandleVelocity,
        const hardware_interface::JointHandle &jointHandleEffort,
        std::size_t jointId) {
    ROS_INFO_STREAM("[RoboclawController::registerJointLimits] jointId: "
                    << jointId
                    << ": name: "
                    << jointNames_[jointId]);
    // Default values
    jointPositionLowerLimits_[jointId] = -std::numeric_limits<double>::max();
    jointPositionUpperLimits_[jointId] = std::numeric_limits<double>::max();
    jointVelocityLimits_[jointId] = std::numeric_limits<double>::max();
    jointEffortLimits_[jointId] = std::numeric_limits<double>::max();

    // Limits datastructures
    joint_limits_interface::JointLimits jointLimits;     // Position
    joint_limits_interface::SoftJointLimits softLimits;  // Soft Position
    bool hasJointLimits = false;
    bool hasSoftLimits = false;

    // Get limits from URDF
    if (model_ == NULL) {
        ROS_ERROR_STREAM("[RoboclawController::registerJointLimits] No URDF model loaded, unable to get joint limits");
        return;
    }

    // Get limits from URDF
    urdf::JointConstSharedPtr urdfJoint = model_->getJoint(jointNames_[jointId]);

    // Get main joint limits
    if (urdfJoint == NULL)  {
        ROS_ERROR_STREAM("[RoboclawController::registerJointLimits] URDF joint not found " << jointNames_[jointId]);
        return;
    }

    // Get limits from URDF
    if (joint_limits_interface::getJointLimits(urdfJoint, jointLimits)) {
        hasJointLimits = true;
        if (jointLimits.has_position_limits) {
            ROS_INFO_STREAM("[RoboclawController::registerJointLimits] Joint "
                            << jointNames_[jointId]
                            << " has URDF position limits ["
                            << jointLimits.min_position
                            << ", "
                            << jointLimits.max_position
                            << "]");
        }

        if (jointLimits.has_velocity_limits) {
            ROS_INFO_STREAM("[RoboclawController::registerJointLimits] Joint "
                            << jointNames_[jointId]
                            << " has URDF velocity limit ["
                            << jointLimits.max_velocity
                            << "]");
        }
    } else {
        if (urdfJoint->type != urdf::Joint::CONTINUOUS) {
            ROS_WARN_STREAM("[RoboclawController::registerJointLimits] Joint "
                            << jointNames_[jointId]
                            << " does not have a URDF "
                            "position limit");
        } else {
            ROS_INFO_STREAM("[RoboclawController::registerJointLimits] Joint "
                            << jointNames_[jointId]
                            << " is a CONTINUOUS joint type");
        }
    }

    // Get limits from ROS param
    if (useRosparamJointLimits_) {
        if (joint_limits_interface::getJointLimits(jointNames_[jointId], nh_, jointLimits)) {
            hasJointLimits = true;
            ROS_INFO_STREAM("[RoboclawController::registerJointLimits] Joint "
                            << jointNames_[jointId]
                            << " has rosparam position limits ["
                            << jointLimits.min_position
                            << ", "
                            << jointLimits.max_position
                            << "]");
            if (jointLimits.has_velocity_limits)
                ROS_INFO_STREAM("[RoboclawController::registerJointLimits] Joint "
                                << jointNames_[jointId]
                                << " has rosparam velocity limit ["
                                << jointLimits.max_velocity
                                << "]");
        }
    } else {
        ROS_INFO_STREAM("[RoboclawController::registerJointLimits] useRosparamJointLimits_ is FALSE");
    }

    // Get soft limits from URDF
    if (useSoftLimitsIfAvailable_) {
        if (joint_limits_interface::getSoftJointLimits(urdfJoint, softLimits)) {
            hasSoftLimits = true;
            ROS_INFO_STREAM("[RoboclawController::registerJointLimits] Joint "
                            << jointNames_[jointId]
                            << " has soft joint limits.");
        } else {
            ROS_INFO_STREAM("[RoboclawController::registerJointLimits] Joint "
                            << jointNames_[jointId]
                            << " does not have soft joint "
                            "limits");
        }
    } else {
        ROS_INFO_STREAM("[RoboclawController::registerJointLimits] useSoftLimitsIfAvailable_ is FALSE");
    }

    // Quit we we haven't found any limits in URDF or rosparam server
    if (!hasJointLimits) {
        ROS_INFO_STREAM("[RoboclawController::registerJointLimits] Has no joint limits");
        return;
    }

    // Copy position limits if available
    if (jointLimits.has_position_limits) {
        // Slighly reduce the joint limits to prevent floating point errors
        jointLimits.min_position += std::numeric_limits<double>::epsilon();
        jointLimits.max_position -= std::numeric_limits<double>::epsilon();

        jointPositionLowerLimits_[jointId] = jointLimits.min_position;
        jointPositionUpperLimits_[jointId] = jointLimits.max_position;
    }

    // Copy velocity limits if available
    if (jointLimits.has_velocity_limits) {
        jointVelocityLimits_[jointId] = jointLimits.max_velocity;
    }

    // Copy effort limits if available
    if (jointLimits.has_effort_limits) {
        jointEffortLimits_[jointId] = jointLimits.max_effort;
    }

    if (hasSoftLimits) {
        // Use soft limits
        ROS_INFO_STREAM("[RoboclawController::registerJointLimits] Using soft saturation limits");
        const joint_limits_interface::PositionJointSoftLimitsHandle softHandlePosition(jointHandlePosition, jointLimits, softLimits);
        positionJointSoftLimits_.registerHandle(softHandlePosition);
        const joint_limits_interface::VelocityJointSoftLimitsHandle softHandleVelocity(jointHandleVelocity, jointLimits, softLimits);
        velocityJointSoftLimits_.registerHandle(softHandleVelocity);
        const joint_limits_interface::EffortJointSoftLimitsHandle softHandleEffort(jointHandleEffort, jointLimits, softLimits);
        effortJointSoftLimits_.registerHandle(softHandleEffort);
    } else {
        // Use saturation limits
        ROS_INFO_STREAM("[RoboclawController::registerJointLimits] Using saturation limits (not soft limits)");
        const joint_limits_interface::PositionJointSaturationHandle satHandlePosition(jointHandlePosition, jointLimits);
        positionJointSaturationInterface_.registerHandle(satHandlePosition);

        const joint_limits_interface::VelocityJointSaturationHandle satHandleVelocity(jointHandleVelocity, jointLimits);
        velocityJointSaturationInterface_.registerHandle(satHandleVelocity);

        const joint_limits_interface::EffortJointSaturationHandle satHandleEffort(jointHandleEffort, jointLimits);
        effortJointSaturationInterface_.registerHandle(satHandleEffort);
    }
}

void RoboclawController::read(const ros::Time& time, const ros::Duration& period) {
}

void RoboclawController::reset() {
    // Reset joint limits state, in case of mode switch or e-stop
    ROS_INFO_STREAM("[RoboclawController::reset]");
    positionJointSaturationInterface_.reset();
    positionJointSoftLimits_.reset();
}


void RoboclawController::write(const ros::Time& time, const ros::Duration& period) {
}
