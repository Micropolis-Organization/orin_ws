#include "IWebotsAckDrive.hpp"
using namespace std;

IWebotsAckDrive::IWebotsAckDrive(ros::NodeHandle &nh, std::string modelName, float wheelBase, float trackWidth, float wheelRadius, std::vector<std::string> driveMotors, std::vector<std::string> steeringMotors, float maxSteeringAngle, float minSteeringAngle)
    : _modelName(modelName),
      _wheelBase(wheelBase),
      _trackWidth(trackWidth),
      _wheelRadius(wheelRadius),
      _driveMotors(driveMotors),
      _steeringMotors(steeringMotors),
      _maxSteeringAngle(maxSteeringAngle),
      _minSteeringAngle(minSteeringAngle)
{
   // set the vector size with respect to the size of the drive and steering motors
   this->_setDrivePosSrv.resize(driveMotors.size());
   this->_omegaWheel.resize(this->_driveMotors.size());
   this->_setDriveVelSrv.resize(driveMotors.size());
   this->_setSteeringPosSrv.resize(steeringMotors.size());

   for (uint8_t i = 0; i < this->_driveMotors.size(); i++)
   {
      std::string cfg_name = this->_modelName + "/" + this->_driveMotors[i];
      this->_driveMotorPosVel.push_back({nh.serviceClient<webots_ros::set_float>(cfg_name + "/set_position"), nh.serviceClient<webots_ros::set_float>(cfg_name + "/set_velocity")});

      this->_setDrivePosSrv[i].request.value = INFINITY;
      auto drivePosStatus = this->_driveMotorPosVel[i].first.call(this->_setDrivePosSrv[i]);
      auto drivePosResponse = this->_setDrivePosSrv[i].response.success;

      if (!drivePosStatus || !drivePosResponse)
      {
         ROS_ERROR("[IWebotsAckDrive] Failed to call service set_position or set_velocity for drive motor #%d.", i + 1);
      }
   }

   for (auto steeringMotor : _steeringMotors)
   {
      std::string cfg_name = this->_modelName + "/" + steeringMotor;
      this->_steeringMotorsClients.push_back(nh.serviceClient<webots_ros::set_float>(cfg_name + "/set_position"));
   }
}

bool IWebotsAckDrive::updateVelocity(float velocity)
{
   this->_currentSpeed = velocity;
   // ROS_INFO("current speed =  %f", this->_currentSpeed);
   for (int i = 0; i < this->_driveMotors.size(); i++)
   {
      if (_setDriveVelSrv.size() == 2)
      { // drive speed
         this->_omegaWheel[i] = (this->_currentSpeed / this->_wheelRadius);
         this->_setDriveVelSrv[i].request.value = this->_omegaWheel[i];
      }
      else if (_setDriveVelSrv.size() == 4)
      {
         this->_omegaWheel[i] = (this->_currentSpeed / this->_wheelRadius);
         this->_setDriveVelSrv[i].request.value = this->_omegaWheel[i];
         // TODO
      }
      auto driveVelStatus = this->_driveMotorPosVel[i].second.call(this->_setDriveVelSrv[i]);
      auto driveVelResponse = this->_setDriveVelSrv[i].response.success;

      if (!driveVelStatus || !driveVelResponse)
      {
         ROS_ERROR("[IWebotsAckDrive] Failed to call service set_position or set_velocity for drive motor #%d.", i + 1);
         return false;
      }
   }
   return true;
}

bool IWebotsAckDrive::updateSteering(float steering)
{
   this->_currentSteering = steering;
   this->_currentSteering = constrain(this->_currentSteering, this->_minSteeringAngle, this->_maxSteeringAngle);

   this->_deltaLeft = atan(this->_wheelBase * tan(this->_currentSteering) / (_wheelBase - 0.5 * this->_trackWidth * tan(this->_currentSteering)));
   this->_deltaRight = atan(this->_wheelBase * tan(this->_currentSteering) / (_wheelBase + 0.5 * this->_trackWidth * tan(this->_currentSteering)));
   // ROS_INFO("[IWebotsAckDrive] ack_angle: %f - left_angle: %f - right_angle: %f ", this->_currentSteering, this->_deltaLeft, this->_deltaRight);

   for (int i = 0; i < this->_steeringMotors.size(); i++)
   {
      this->_setSteeringPosSrv[i].request.value = i & 1 ? _deltaLeft : _deltaRight;
      if (i > 1)
         this->_setSteeringPosSrv[i].request.value *= -1;
      auto steeringPosStatus = this->_steeringMotorsClients[i].call(this->_setSteeringPosSrv[i]);
      auto steeringPosResponse = this->_setSteeringPosSrv[i].response.success;

      if (!steeringPosStatus || !steeringPosResponse)
      {
         ROS_ERROR("[IWebotsAckDrive] Failed to call service set_position for steering motor #%d.", i + 1);
         return false;
      }
   }
   return true;
}

float IWebotsAckDrive::constrain(float val, float minValue, float maxValue)
{
   return min(maxValue, max(val, minValue));
}
IWebotsAckDrive::~IWebotsAckDrive()
{
   ros::shutdown();
}
