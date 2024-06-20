#include "iWebotsNode.hpp"

iWebotsNode::iWebotsNode(std::pair<int, int> numMotors) // # drive motors , # steering motors
{
   this->_nh = new ros::NodeHandle();
   this->_modelNameSub = this->_nh->subscribe<std_msgs::String>("model_name", 1, [&](const std_msgs::String::ConstPtr &modelNameMsg)
                                                                { this->_robot_model = modelNameMsg->data; });
   ros::Rate r(10.0);

   while (this->_robot_model.empty())
   { // this loop is only used to wait until reading from the model name topic
      ROS_ERROR("[IWebotsNode] Robot Name: Empty");
      ros::spinOnce();
      r.sleep();
   }
   ROS_INFO("[IWebotsNode] Robot Name: %s ", this->_robot_model.c_str());
   // std::vector<std::string> _steeringMotors;

   this->_driveMotors.resize(numMotors.first);
   this->_steeringMotors.resize(numMotors.second);

   this->getRosParam(this->_robot_model + "/time_step", this->_timeStep);

   this->getRosParam(this->_robot_model + "/odom_topic_out", this->_odomTopicOut);
   this->getRosParam(this->_robot_model + "/linear_vel_cmd", this->_linearVelCmdTopic);
   this->getRosParam(this->_robot_model + "/steering_cmd", this->_steeringCmdTopic);

   this->getRosParam(this->_robot_model + "/odom_frame", this->_odomFrame);
   this->getRosParam(this->_robot_model + "/base_frame", this->_baseFrame);

   this->getRosParam(this->_robot_model + "/wheel_base", this->_wheelBase);
   this->getRosParam(this->_robot_model + "/track_width", this->_trackWidth);
   this->getRosParam(this->_robot_model + "/wheel_radius", this->_wheelRadius);

   this->getRosParam(this->_robot_model + "/min_steering_angle", this->_minSteeringAngle);
   this->getRosParam(this->_robot_model + "/max_steering_angle", this->_maxSteeringAngle);

   for (uint8_t i = 0; i < numMotors.first; i++)
   {
      std::stringstream stream;
      stream << (i + 1);
      std::string s = stream.str();
      this->getRosParam(this->_robot_model + "/drive_motor_" + s, this->_driveMotors[i]);
   }

   for (uint8_t i = 0; i < numMotors.second; i++)
   {
      std::stringstream stream;
      stream << (i + 1);
      std::string s = stream.str();
      this->getRosParam(this->_robot_model + "/steering_motor_" + s, this->_steeringMotors[i]);
   }

   this->_velocitySub = this->_nh->subscribe<std_msgs::Float32>(this->_linearVelCmdTopic, 1, [&](const std_msgs::Float32::ConstPtr &velocityMsg)
                                                                { this->_linearVel = velocityMsg->data; });

   this->_steeringSub = this->_nh->subscribe<std_msgs::Float32>(this->_steeringCmdTopic, 1, [&](const std_msgs::Float32::ConstPtr &steeringMsg)
                                                                { this->_steeringAng = steeringMsg->data; });

   this->_odomPub = this->_nh->advertise<nav_msgs::Odometry>(this->_odomTopicOut, 1);

   setTimeStep();
   enablePCL();
   this->_odom = new IWebotsOdom(this->_robot_model, this->_odomFrame, this->_baseFrame, this->_odomTopicOut);
   this->_iWebotsAckDrive = new IWebotsAckDrive(*this->_nh, this->_robot_model, this->_wheelBase, this->_trackWidth, this->_wheelRadius, this->_driveMotors, this->_steeringMotors, this->_maxSteeringAngle, this->_minSteeringAngle);
}
uint64_t prev_time_ms = 0;
uint64_t current_time_ms = 0;

void iWebotsNode::updateLoop()
{
   if (!this->_iWebotsAckDrive->updateVelocity(this->_linearVel))
   {
      ROS_ERROR("[IWebotsNode] Failed to update velocity.");
   }
   if (!this->_iWebotsAckDrive->updateSteering(this->_steeringAng))
   {
      ROS_ERROR("[IWebotsNode] Failed to update steering.");
   }
   if (!this->_odom->updateOdom())
   {
      ROS_ERROR("[IWebotsNode] Failed to update Odom and TF.");
      exit(1);
   }

   // ros::Time current_time = ros::Time::now();
   // prev_time_ms = current_time_ms;
   // current_time_ms = current_time.toNSec() / 1000000;
   // uint16_t ms = current_time_ms % 1000;
   // uint16_t result = current_time_ms -  prev_time_ms;

   // Print the current time in milliseconds
   // ROS_INFO("Current ROS time : %lu", current_time_ms);
   // ROS_INFO("Current ROS time (ms) : %lu", ms);
   // ROS_INFO("result: %lu", result);

}

// void iWebotsNode::robotStatus(bool enableRobotStatus)
// {
//     if (!enableRobotStatus)
//         return;

//     printf("\033[2J\033[1;1H");
//     std::cout << "====* Autonomous Driving SW Team *====" << std::endl;
//     std::cout << "============ Robot Status ============" << std::endl
//                 << std::endl
//                 << std::setprecision(3) << std::fixed;

//     std::cout << "Velocity          = " << this->_odomWithTf.first.twist.twist.linear.x << " m/s" << std::endl;
//     std::cout << "Ack. Angle        = " << this->_steeringCmd * 180 / M_PI << " deg" << std::endl;

//     std::cout << "Position    [xyz] = " << this->_odomWithTf.first.pose.pose.position.x << ", " << this->_odomWithTf.first.pose.pose.position.y << ", " << this->_odomWithTf.first.pose.pose.position.z << " m" << std::endl;

//     std::cout << "Orientation [yaw] = " << _iWebotsOdom->yawFromQuaternion(_odomWithTf.first.pose.pose.orientation) * (180/M_PI) << " degs" << std::endl ;
//     std::cout << "======================================" << std::endl ;

// }

void iWebotsNode::setTimeStep()
{
   this->setTimeStepClient = this->_nh->serviceClient<webots_ros::set_int>(this->_robot_model + "/robot/time_step");
   this->timeStep_Srv.request.value = this->_timeStep;

   if (!setTimeStepClient.call(timeStep_Srv) || !timeStep_Srv.response.success)
   {
      ROS_ERROR("[IWebotsNode] Failed to call time_step service.");
      exit(1);
   }
   ROS_INFO("[IWebotsNode] Time set done");
}

void iWebotsNode::enablePCL()
{
   this->enablePCLClient = this->_nh->serviceClient<webots_ros::set_bool>(this->_robot_model + "/lidar/enable_point_cloud");
   this->enablePclSrv.request.value = true;

   if (!this->enablePCLClient.call(enablePclSrv) || !this->enablePclSrv.response.success)
   {
      ROS_ERROR("[IWebotsNode] Failed to enable PointCloud service.");
      exit(1);
   }
   ROS_INFO("[IWebotsNode] PointCloud is enabled");
}

void iWebotsNode::getRosParam(std::string paramName, auto &paramValue)
{
   if (this->_nh->getParam(paramName, paramValue))
   {
      std::stringstream strg;
      strg << paramValue;
      std::string s = strg.str();
      ROS_INFO("[IWebotsRosNode] [PARAM] %s = %s", paramName.c_str(), s.c_str());
   }
   else
   {
      ROS_WARN("[IWebotsRosNode] [PARAM] %s is not set", paramName.c_str());
   }
}
iWebotsNode::~iWebotsNode()
{
   ROS_INFO("[IWebotsNode] ShutDown");
   ros::shutdown();
   exit(1);
}