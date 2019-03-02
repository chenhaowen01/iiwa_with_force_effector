#!/usr/bin/env python

import sys
import rospy
import actionlib
from iiwa_msgs.msg import CartesianPose, MoveToCartesianPoseAction, MoveToCartesianPoseActionGoal

g_current_cartesian_pose = None

def cartesian_pose_callback(pose):
    global g_current_cartesian_pose
    # rospy.loginfo(pose)
    g_current_cartesian_pose = pose

def main():
    global g_current_cartesian_pose

    rospy.init_node('test_cartesian_pose_lin', anonymous=True)
    rospy.Subscriber('state/CartesianPose', CartesianPose, cartesian_pose_callback)
    cartesian_pose_lin_client = actionlib.SimpleActionClient('test', MoveToCartesianPoseAction)

    while not g_current_cartesian_pose:
        rospy.loginfo('wait for current pose!')
        rospy.sleep(1)

    rospy.loginfo(g_current_cartesian_pose)

    p = g_current_cartesian_pose
    p.poseStamped.pose.position.x -= 0.02
    rospy.loginfo(p)

    rospy.loginfo('wait for action server...')
    if (cartesian_pose_lin_client.wait_for_server(rospy.Duration(5))):
        goal = MoveToCartesianPoseActionGoal()
        goal.goal.cartesian_pose = p
        rospy.loginfo(goal)
        cartesian_pose_lin_client.send_goal(goal)
    else:
        rospy.loginfo('wait action server timeout')

if __name__ == "__main__":
    main()