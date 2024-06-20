#ifndef _IWEBOTSNODE_HPP_
#define _IWEBOTSNODE_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include "IWebotsAckDrive.hpp"
#include "IWebotsOdom.hpp"

class iWebotsNode
{
private:
    ros::ServiceClient setTimeStepClient;
    webots_ros::set_int timeStep_Srv;
    ros::ServiceClient enablePCLClient;
    webots_ros::set_bool enablePclSrv;

    ros::Subscriber _modelNameSub,
        _velocitySub,
        _steeringSub;
    ros::Publisher _odomPub;
    ros::NodeHandle *_nh;

    int _timeStep = 0;

    std::vector<std::string> _driveMotors, _steeringMotors;
    std::string _robot_model,
        _odomTopicOut,
        _linearVelCmdTopic,
        _steeringCmdTopic,
        _odomFrame,
        _baseFrame;

    float _linearVel = 0.0,
          _steeringAng = 0.0,
          _wheelBase,
          _trackWidth,
          _wheelRadius,
          _minSteeringAngle,
          _maxSteeringAngle;

    IWebotsAckDrive *_iWebotsAckDrive;
    IWebotsOdom *_odom;

    void getRosParam(std::string paramName, auto &paramValue);
    void setTimeStep();
    void enablePCL();
    void robotStatus(bool enableRobotStatus);

public:
    void updateLoop();
    iWebotsNode(std::pair<int, int> numMotors);
    ~iWebotsNode();
};

#endif //_IWEBOTSNODE_HPP_