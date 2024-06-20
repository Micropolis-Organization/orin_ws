#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <visualization_msgs/MarkerArray.h> 

#define NO_GOAL_PTS 6

using namespace std;
using namespace ros;

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;




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


void create_path_callback(const visualization_msgs::MarkerArray::ConstPtr& msg){

   vector<geometry_msgs::Pose> Points;
   geometry_msgs::Pose point;

   // Check if this is the first message (which is the dummy message to be neglected)
   if (msg->markers.size() == 1) {
      // Skip processing the message with only one marker
      return;
   }

   // Create the path 
   
   float marker_x;
   float marker_y;
   float marker_qx;
   float marker_qy;
   float marker_qz;
   float marker_qw;

   for (auto it = msg->markers.begin()+1 ; it != msg->markers.end() ; ++it) {

      // the marker object
      const auto& marker = *it;

      // marker type
      // const type_info& markerType = typeid(marker);

      // print the marker x position
      marker_x =  marker.pose.position.x;
      marker_y =  marker.pose.position.y;
      marker_qx = marker.pose.orientation.x;
      marker_qy = marker.pose.orientation.y;
      marker_qz = marker.pose.orientation.z;
      marker_qw = marker.pose.orientation.w;

      cout << "New marker" << endl;
      cout << marker_x << endl;
      cout << marker_y << endl;
      cout << marker_qx << endl;
      cout << marker_qy << endl;
      cout << marker_qz << endl;
      cout << marker_qw << endl;

      point.position.x = marker_x;
      point.position.y = marker_y;
      point.orientation.x = marker_qx;
      point.orientation.y = marker_qy;
      point.orientation.z = marker_qz;
      point.orientation.w = marker_qw;
      Points.push_back(point);
   }

   cout << "   Waypoints DONE       " << endl;


   // for (uint16_t i = 0; i < Points.size(); i++)
   // {
   //    goal.target_pose.header.frame_id = "map";
   //    goal.target_pose.header.stamp = ros::Time::now();
   //    goal.target_pose.pose = Points[i];

   //    ROS_INFO("Sending goal %d", i+1);
   //    ac.sendGoal(goal);

   //    ac.waitForResult();

   //    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
   //    {
   //       ROS_INFO("The robot has arrived at the goal %d", i);
   //    }
   //    else
   //    {
   //       ROS_INFO("The base failed to move forward 1 meter for some reason");
   //    }
   // }


}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "waypoints_navigation");
   ros::NodeHandle nh;
   ros::Publisher goalPub;
   MoveBaseClient ac("move_base", true);
   move_base_msgs::MoveBaseGoal goal;


   // ros::topic::waitForMessage<visualization_msgs::MarkerArray>("/sPathVehicle", create_path)
   
   ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>("/sPathVehicle", 1, create_path_callback);

   ros::spin();

   // mrk_ar  = *(ros::topic::waitForMessage<visualization_msgs::MarkerArray>(/sPathVehicle,ros::Duration(20)));



}
