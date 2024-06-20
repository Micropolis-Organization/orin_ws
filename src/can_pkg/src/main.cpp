#include "ROS_Node.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CAN_node");
  ROS_Node ros_node;
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    
    ros_node.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
}
