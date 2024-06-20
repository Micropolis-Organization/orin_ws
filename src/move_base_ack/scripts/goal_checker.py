#!/usr/bin/env python3

import rospy
from nav_msgs.srv import GetPlan , GetPlanRequest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Path

start = PoseStamped()
start.header.frame_id = "map"
start.pose.position.x = 41.29
start.pose.position.y = 1.166
start.pose.position.z = 0.2055
start.pose.orientation.x = 0.0
start.pose.orientation.y = 0.0
start.pose.orientation.z = 0.03487771456112561
start.pose.orientation.w = 1.0

      
goal = PoseStamped()
goal.header.frame_id = "map"
goal.pose.position.x = 30
goal.pose.position.y = 5
goal.pose.position.z = 0.0
goal.pose.orientation.x = 0.0
goal.pose.orientation.y = 0.0
goal.pose.orientation.z = 0.5
goal.pose.orientation.w = 0.8

tolerance = 1.0

 
if __name__ == "__main__":
    rospy.init_node("goal_checker")
    rospy.wait_for_service('/move_base/make_plan')
    rospy.loginfo("Service is available")
    
    getPlanSrv = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    plan = GetPlanRequest(start,goal,tolerance)
    
    

    get_path = getPlanSrv(plan)
    get_path.plan = Path()
    print ("Number of poses: %d" %(len(get_path.plan.poses)))
    
