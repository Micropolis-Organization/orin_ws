#ifndef _IWEBOTS_ACKDRIVE_HPP_
#define _IWEBOTS_ACKDRIVE_HPP_

#include <webots_ros/set_float.h>
#include <ros/ros.h>
class IWebotsAckDrive
{
private:
    std::string _modelName;

    std::vector<std::pair<ros::ServiceClient, ros::ServiceClient>> _driveMotorPosVel; // pos and vel
    std::vector<ros::ServiceClient> _steeringMotorsClients;
    std::vector<webots_ros::set_float> _setDrivePosSrv;
    std::vector<webots_ros::set_float> _setDriveVelSrv;
    std::vector<webots_ros::set_float> _setSteeringPosSrv;

    std::vector<std::string> _driveMotors;
    std::vector<std::string> _steeringMotors;
    std::vector<float> _omegaWheel;

    float _wheelBase,
        _trackWidth,
        _wheelRadius;
    float _deltaLeft,
        _deltaRight;
    float _currentSpeed = 0.0,
        _currentSteering = 0.0,
        _currentOmegaZ,
        _maxSteeringAngle,
        _minSteeringAngle;

    float
    constrain(float val, float min, float max);

public:
    bool updateVelocity(float velocity);
    bool updateSteering(float steering);
    IWebotsAckDrive(ros::NodeHandle &nh, std::string modelName, float wheelBase, float trackWidth, float wheelRadius, std::vector<std::string> driveMotors, std::vector<std::string> steeringMotors, float maxSteeringAngle, float minSteeringAngle);
    ~IWebotsAckDrive();
};

#endif