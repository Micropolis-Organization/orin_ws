#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#define NO_GOAL_PTS 6

using namespace std;
using namespace ros;

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

vector<geometry_msgs::Pose> Points;

void checkParam(const std::string &param_name, auto &param_value)
{
   if (!param::get(param_name, param_value))
   {
      ROS_WARN("[PARAM] %s is not set", param_name.c_str());
      return;
   }

   std::stringstream ss;
   ss << param_value;
   std::string param_value_str = ss.str();

   ROS_INFO("[PARAM] %s = %s", param_name.c_str(), param_value_str.c_str());
};
void callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
}
int main(int argc, char **argv)
{
   ros::init(argc, argv, "waypoints_navigation");
   ros::NodeHandle nh;
   ros::Publisher goalPub;
   ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>("/pathVehicle", 10, callback);
   // MoveBaseClient ac("move_base", true);

   // Wait for the action server to come up so that we can begin processing goals.
   // while (!ac.waitForServer(ros::Duration(5.0)))
   // {
   //    ROS_INFO("Waiting for the move_base action server to come up");
   // }
   move_base_msgs::MoveBaseGoal goal;

   for (uint16_t i = 1; i <= NO_GOAL_PTS; i++)
   {
      geometry_msgs::Pose point;
      string point_name = "p" + to_string(i);
      checkParam("/waypoints/" + point_name + "/position/x", point.position.x);
      checkParam("/waypoints/" + point_name + "/position/y", point.position.y);
      checkParam("/waypoints/" + point_name + "/orientation/x", point.orientation.x);
      checkParam("/waypoints/" + point_name + "/orientation/y", point.orientation.y);
      checkParam("/waypoints/" + point_name + "/orientation/z", point.orientation.z);
      checkParam("/waypoints/" + point_name + "/orientation/w", point.orientation.w);
      Points.push_back(point);
      cout << "          " << endl;
   }
   cout << "   Waypoints DONE       " << endl;

   for (uint16_t i = 0; i < Points.size(); i++)
   {
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = Points[i];

      ROS_INFO("Sending goal %d", i + 1);
      // ac.sendGoal(goal);

      // ac.waitForResult();

      // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      // {
      //    ROS_INFO("The robot has arrived at the goal %d", i);
      // }
      // else
      // {
      //    ROS_INFO("The base failed to move forward 1 meter for some reason");
      // }
   }
}
