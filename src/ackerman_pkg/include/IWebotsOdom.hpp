#ifndef _IWEBOTSODOM_HPP_
#define _IWEBOTSODOM_HPP_

#include <ros/ros.h>
#include <webots_ros/get_uint64.h>
#include <webots_ros/node_get_position.h>
#include <webots_ros/node_get_orientation.h>
#include <webots_ros/node_get_velocity.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
class IWebotsOdom
{
private:
   ros::NodeHandle *_nh;
   ros::Publisher _odomPub;
   nav_msgs::Odometry _odomMsg;
   geometry_msgs::TransformStamped _tf;
   tf::TransformBroadcaster _tfBroadcaster;

   geometry_msgs::Pose _currentPose;
   geometry_msgs::Twist _currentTwist;
   string _modelName,
       _odomTopic,
       _odomFrame,
       _baseFrame;

   pair<ros::ServiceClient, webots_ros::get_uint64> _robotIDSrvClient;
   pair<ros::ServiceClient, webots_ros::node_get_position> _robotPosClient;
   pair<ros::ServiceClient, webots_ros::node_get_velocity> _robotVelClient;
   pair<ros::ServiceClient, webots_ros::node_get_orientation> _robotOrientClient;

   ros::ServiceClient _robotOrientCli;
   webots_ros::node_get_orientation _robotOrientSrv;
   uint64_t _robotID = 0;
   double getYawFromQuat(geometry_msgs::Quaternion quat);

public:
   bool updateOdom();

   IWebotsOdom(string modelName, string odomFrame, string baseFrame, string odomTopic);
   ~IWebotsOdom();
};

#endif