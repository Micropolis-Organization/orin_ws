#include "IWebotsOdom.hpp"

IWebotsOdom::IWebotsOdom(string modelName, string odomFrame, string baseFrame, string odomTopic)
    : _modelName(modelName),
      _odomFrame(odomFrame),
      _baseFrame(baseFrame),
      _odomTopic(odomTopic)

{
   this->_nh = new ros::NodeHandle();
   this->_robotIDSrvClient.first = this->_nh->serviceClient<webots_ros::get_uint64>(this->_modelName + "/supervisor/get_self");
   if (this->_robotIDSrvClient.first.call(this->_robotIDSrvClient.second))
   {
      this->_robotID = this->_robotIDSrvClient.second.response.value;
      ROS_INFO("Robot ID: %ld", this->_robotID);
   }
   else
   {
      ROS_ERROR("[IWebotsOdom] Failed to call service get_self.");
      exit(1);
   }
   string srv_name = this->_modelName + "/supervisor/node/";
   this->_robotPosClient.first = this->_nh->serviceClient<webots_ros::node_get_position>(srv_name + "get_position");
   this->_robotPosClient.second.request.node = this->_robotID;

   this->_robotVelClient.first = this->_nh->serviceClient<webots_ros::node_get_velocity>(srv_name + "get_velocity");
   this->_robotVelClient.second.request.node = this->_robotID;

   this->_robotOrientCli = this->_nh->serviceClient<webots_ros::node_get_orientation>(srv_name + "get_orientation");
   this->_robotOrientClient.second.request.node = this->_robotID;

   this->_odomPub = this->_nh->advertise<nav_msgs::Odometry>(this->_odomTopic, 1);
}

double IWebotsOdom::getYawFromQuat(geometry_msgs::Quaternion quat)
{
   tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   return yaw;
}
bool IWebotsOdom::updateOdom()
{
   if (this->_robotPosClient.first.call(this->_robotPosClient.second))
   {
      this->_currentPose.position.x = this->_robotPosClient.second.response.position.x;
      this->_currentPose.position.y = this->_robotPosClient.second.response.position.y;
      this->_currentPose.position.z = this->_robotPosClient.second.response.position.z;

      // ROS_INFO("POS x: %f", this->_currentPose.position.x);
      // ROS_INFO("POS y: %f", this->_currentPose.position.y);
      // ROS_INFO("POS z: %f", this->_currentPose.position.z);
      // ROS_INFO("----------------");
   }
   else
   {
      ROS_ERROR("[IWebotsOdom] Failed to call get_position service.");
      return false;
   }
   if (this->_robotOrientCli.call(this->_robotOrientClient.second))
   {
      this->_currentPose.orientation.x = this->_robotOrientClient.second.response.orientation.x;
      this->_currentPose.orientation.y = this->_robotOrientClient.second.response.orientation.y;
      this->_currentPose.orientation.z = this->_robotOrientClient.second.response.orientation.z;
      this->_currentPose.orientation.w = this->_robotOrientClient.second.response.orientation.w;
      // double yaw = this->getYawFromQuat(this->_currentPose.orientation);
      // ROS_INFO("z  : %f", this->_robotOrientClient.second.response.orientation.z);
      // ROS_INFO("w  : %f", this->_robotOrientClient.second.response.orientation.w);
      // ROS_INFO("Yaw: %f", yaw);
      // ROS_INFO("------------------");
   }
   else
   {
      ROS_ERROR("[IWebotsOdom] Failed to call get_Orientation service.");
      return false;
   }
   if (this->_robotVelClient.first.call(this->_robotVelClient.second))
   {
      this->_currentTwist.linear.x = this->_robotVelClient.second.response.velocity.linear.x;
      this->_currentTwist.angular.z = this->_robotVelClient.second.response.velocity.angular.z;
      // ROS_INFO("Vel x: %f", this->_currentTwist.linear.x);
      // ROS_INFO("Vel z: %f", this->_currentTwist.angular.z);
      // ROS_INFO("----------------");
   }
   else
   {
      ROS_ERROR("[IWebotsOdom] Failed to call get_velocity service.");
      return false;
   }

   this->_odomMsg.header.stamp = ros::Time::now();
   this->_odomMsg.header.frame_id = this->_odomFrame;
   this->_odomMsg.child_frame_id = this->_baseFrame;
   this->_odomMsg.pose.pose.position = this->_currentPose.position;
   this->_odomMsg.pose.pose.orientation = this->_currentPose.orientation;
   this->_odomMsg.twist.twist.linear = this->_currentTwist.linear;
   this->_odomMsg.twist.twist.angular = this->_currentTwist.angular;
   this->_odomPub.publish(this->_odomMsg);

   this->_tf.header.stamp = _odomMsg.header.stamp;
   this->_tf.header.frame_id = this->_odomFrame;
   this->_tf.child_frame_id = this->_baseFrame;
   this->_tf.transform.translation.x = this->_currentPose.position.x;
   this->_tf.transform.translation.y = this->_currentPose.position.y;
   this->_tf.transform.translation.z = this->_currentPose.position.z;
   this->_tf.transform.rotation = this->_currentPose.orientation;

   this->_tfBroadcaster.sendTransform(this->_tf);
   return true;
}
IWebotsOdom::~IWebotsOdom()
{
}
