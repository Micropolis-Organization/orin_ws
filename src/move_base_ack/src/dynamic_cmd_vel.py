#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_dynamic(cmd_int: Twist):

    vel = Twist()

    # Get the value of the dummy parameter
    dynamic_flag_value = rospy.get_param('/dynamic_flag')

    if dynamic_flag_value == False:
        vel = cmd_int
        cdm_vel_pub.publish(vel)
    else:
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 0.0
        cdm_vel_pub.publish(vel)

        rospy.sleep(2)
        rospy.set_param('/dynamic_flag', False)
    


if __name__ == "__main__":
    rospy.init_node("my_cmd_vel_publisher")
    
    cdm_vel_int_sub = rospy.Subscriber("/cmd_vel_intermediate", Twist, cmd_vel_dynamic)
    cdm_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size= 5)
    rospy.spin()