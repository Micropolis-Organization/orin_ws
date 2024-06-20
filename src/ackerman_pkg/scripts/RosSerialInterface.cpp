#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace ros;
using namespace std;

float velocity,
    steering_rad;
pair<float, float> cmdVel;

int mapValue(int value, int in_min, int in_max, int out_min, int out_max) {
   value = min(max(value,in_min),in_max);
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "Ros_Serial_Interface", ros::init_options::AnonymousName);
   ros::NodeHandle _nh;
   ros::Subscriber cmdVelSub;
   cmdVelSub = _nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, [&](const geometry_msgs::Twist::ConstPtr &cmdVelMsg)
                                                   { cmdVel.first = cmdVelMsg->linear.x,
                                                     cmdVel.second = cmdVelMsg->angular.z; });
   ros::Rate r(10.0);

   while (ros::ok())
   {
      // transform()
      ros::spinOnce();
      r.sleep();
   }

   ros::shutdown();
   return 0;
}
