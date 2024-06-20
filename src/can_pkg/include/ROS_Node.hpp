#ifndef __ROS_NODE_HPP__
#define __ROS_NODE_HPP__

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "CAN_Interface.hpp"

class ROS_Node
{
private:
      ros::NodeHandle *_nh;

      ros::Subscriber _velocity_sub,
          _steering_sub;

      ros::Publisher _speedFeedback_pub;

      CAN_Interface *can_interface;

      std::string _USB_PORT;

      std_msgs::Float32 _speed_fb_msg;

      enum directionState
      {
            FORWARD,
            BACKWARD,
            IDLE
      };
      std::string directionStateString[3] = {"FORWARD", "BACKWARD", "IDLE"};
      directionState robotDirectionState;
      float _velocity = 50.0,
            _steering = 50.0;
      float _prev_velocity = 50,
            _prev_steering = 50;

      int idle_counter=0;
      float _speed_fb = 0.0;
      void getRosParam(std::string paramName, auto &paramValue);
      void printOnTerminal();
      void directionControl(float &velocity, float &steering);

public:
      ROS_Node(/* args */);
      ~ROS_Node();
      void update();
};

#endif //__ROS_NODE_HPP__