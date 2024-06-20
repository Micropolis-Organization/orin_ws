#include "ROS_Node.hpp"

ROS_Node::ROS_Node(/* args */)
{
   this->_nh = new ros::NodeHandle(); // Create a Node Handle for the ROS

   this->_velocity_sub = this->_nh->subscribe<std_msgs::Float32>("velocity", 1, [&](const std_msgs::Float32::ConstPtr &velocityMsg)
                                                                 {
                                                                    this->_velocity = velocityMsg->data;
                                                                    std::cout<<"velocity: " <<this->_velocity<<std::endl; });

   this->_steering_sub = this->_nh->subscribe<std_msgs::Float32>("steering_rad", 1, [&](const std_msgs::Float32::ConstPtr &steeringMsg)
                                                                 {
                                                                    this->_steering = steeringMsg->data;
                                                                     std::cout<<"steering: " <<this->_steering<<std::endl; });

   this->_speedFeedback_pub = this->_nh->advertise<std_msgs::Float32>("speed_fb", 1);

   this->getRosParam("/can_node/port", this->_USB_PORT);
   can_interface = new CAN_Interface((char *)this->_USB_PORT.c_str());
   this->robotDirectionState = IDLE;
   ROS_INFO("[RosNode] ready to start");
}

void ROS_Node::directionControl(float &velocity, float &steering)
{
   if (robotDirectionState == FORWARD)
   {
      idle_counter = 0;
      if ((velocity < 48 && this->_speed_fb > 0) )
      {
         velocity = steering = 50;
         robotDirectionState = IDLE;
      }
   }
   else if (robotDirectionState == BACKWARD )
   {
      idle_counter = 0;

      if (velocity > 52 && this->_speed_fb < 0)
      {
         velocity = steering = 50;
         robotDirectionState = IDLE;
      }
   }
   else if (robotDirectionState == IDLE)
   {
      // if (this->_speed_fb != 0)
      //    idle_counter = 0;

      if (velocity > 52 && this->_speed_fb == 0)
      {
         idle_counter++;
         velocity = 50;
         if (idle_counter > 10)
            robotDirectionState = FORWARD;
      }
      else if (velocity < 48 && this->_speed_fb == 0)
      {
         idle_counter++;
         velocity = 50;

         if (idle_counter > 10)
            robotDirectionState = BACKWARD;
      }
   }
}

void ROS_Node::update()
{
 //  this->_speed_fb = can_interface->getFeedback();
 //  this->_speed_fb_msg.data = this->_speed_fb;
 //  this->_speedFeedback_pub.publish(this->_speed_fb_msg);

   // printf("loop");
 //  directionControl(this->_velocity, this->_steering);

   can_interface->sendCmdVel(this->_velocity, this->_steering);
   // std::cout<<"speed_fb: " <<this->_speed_fb_msg.data<<std::endl;
   // this->printOnTerminal();
}

void ROS_Node::getRosParam(std::string paramName, auto &paramValue)
{
   if (!this->_nh->getParam(paramName, paramValue))
   {
      ROS_WARN("[RosNode] [PARAM] %s is not set", paramName.c_str());
      exit(1);
   }
   std::stringstream strg;
   strg << paramValue;
   std::string s = strg.str();
   ROS_INFO("[RosNode] [PARAM] %s = %s", paramName.c_str(), s.c_str());
}
void ROS_Node::printOnTerminal()
{
   system("clear");
   std::cout << "---------------------- " << std::endl;
   std::cout << "\t velocity: " << this->_velocity << std::endl;
   std::cout << "\t steering: " << this->_steering << std::endl;
   std::cout << "\t speed_fb: " << this->_speed_fb_msg.data << std::endl;
   std::cout << "\t direction state: " << directionStateString[robotDirectionState] << std::endl;
   std::cout << "\t idle_counter: " << idle_counter << std::endl;
}

ROS_Node::~ROS_Node()
{
   this->_nh->shutdown();
}
