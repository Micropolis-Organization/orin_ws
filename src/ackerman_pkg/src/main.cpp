#include <ros/ros.h>
#include "iWebotsNode.hpp"
int main(int argc, char **argv)
{
   ros::init(argc, argv, "Ackermann_Robot_node", ros::init_options::AnonymousName);
   iWebotsNode I_Webots_Node({4, 4});
   // IWebotsOdom odom;
   // ros::Rate r(30.0);
   while (ros::ok())
   {
      I_Webots_Node.updateLoop();
      // r.sleep();
      ros::spinOnce();
   }

   ros::shutdown();
   return 0;
}
