#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def amcl_cb(amclPose:PoseWithCovarianceStamped):
    """Callback function for AMCL odometry topic."""
    print("-------------")
    odom = Odometry()
    odom.header = amclPose.header
    odom.pose.pose = amclPose.pose.pose
    print("x:",odom.pose.pose.position.x)
    print("y:",odom.pose.pose.position.y)
    odomPublisher.publish(odom)
    

if __name__ == "__main__":
    rospy.init_node("odom_publisher")
    
    amclPoseSub = rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,amcl_cb)
    odomPublisher = rospy.Publisher("robot/Pose",Odometry,queue_size= 5)
    rospy.spin()