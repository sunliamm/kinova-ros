#!/usr/bin/env python

import sys
import rospy
from kinova_manipulation.srv import *
from geometry_msgs.msg import PoseStamped
import copy

def pcik_place_client():

    print "calling the service"

    rospy.wait_for_service('/kinova_manipulation/pick_place')

    print "calling the service"

    try:
        pick_pose = PoseStamped()
        place_pose = PoseStamped()

        pick_pose.pose.position.x = -0.2
        pick_pose.pose.position.y = -0.2
        pick_pose.pose.position.z = -0.1

        place_pose.pose.position.x = 0.5
        place_pose.pose.position.y = -0.1
        place_pose.pose.position.z = 0.2

        pick_type = "VERT"
        gripper_direction = 0.
        offset_minus = 0.02
        offset_plus = 0.05
        is_place = True
        is_home = True

        pick_place = rospy.ServiceProxy('/kinova_manipulation/pick_place', Pick_Place)
        res = pick_place(pick_pose, place_pose, gripper_direction, offset_minus, offset_plus, pick_type, is_place, is_home)
        
        return res.result

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    
    result =  pcik_place_client()
