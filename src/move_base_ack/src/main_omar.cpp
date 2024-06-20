#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace ros;

#include <algorithm>

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher waypoints_pub;

float map_resolution = 0.05;
MoveBaseClient *ac;

void create_path_callback(const visualization_msgs::MarkerArray::ConstPtr &msg)
{

   move_base_msgs::MoveBaseGoal goal;
   vector<geometry_msgs::Pose> Points;
   geometry_msgs::Pose point;

   // Wait for the action server to come up so that we can begin processing goals.
   // while (!ac->waitForServer(ros::Duration(5.0)))
   // {
   //    ROS_INFO("Waiting for the move_base action server to come up");
   // }

   // Check if this is the first message (which is the dummy message to be neglected)
   if (msg->markers.size() == 1)
   {
      // Skip processing the message with only one marker
      return;
   }

   // Create the path of waypoints

   for (auto it = msg->markers.begin() + 1; it != msg->markers.end(); ++it)
   {

      // the marker object
      const auto &marker = *it;

      point.position.x = map_resolution * marker.pose.position.x;
      point.position.y = map_resolution * marker.pose.position.y;
      point.orientation.x = marker.pose.orientation.x;
      point.orientation.y = marker.pose.orientation.y;
      point.orientation.z = marker.pose.orientation.z;
      point.orientation.w = marker.pose.orientation.w;

      // cout << "New waypoint" << endl;
      // cout << "x: " << point.position.x << endl;
      // cout << "y: " << point.position.y << endl;

      Points.push_back(point);
   }

   // Since hybrid_astar gives the waypoints from goal to start, we need to flip them
   reverse(Points.begin(), Points.end());

   for (auto my_point : Points)
   {
      cout << "New waypoint" << endl;
      cout << "x: " << my_point.position.x << endl;
      cout << "y: " << my_point.position.y << endl;
      cout << "z: " << my_point.orientation.z << endl;
      cout << "w: " << my_point.orientation.w << endl;
   }

   // Inform me that waypoints are done
   cout << "   Waypoints DONE       " << endl;

   // Send the waypoints
   for (uint16_t i = 0; i < Points.size(); i++)
   {
      // Publish PoseArray
      waypoints_pub.publish(Points[i]);

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = Points[i];

      ROS_INFO("Sending goal %d", i + 1);
      ac->sendGoal(goal);

      ac->waitForResult();

      if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
         ROS_INFO("The robot has arrived at the goal %d", i);
      }
      else
      {
         ROS_INFO("The base failed to move forward 1 meter for some reason");
      }
   }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "waypoints_navigation");
   ros::NodeHandle nh;
   ac = new MoveBaseClient("move_base", true);

   waypoints_pub = nh.advertise<geometry_msgs::Pose>("/my_waypoints", 10);

   ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>("/sPathVehicle", 1, create_path_callback);

   ros::spin();
}
