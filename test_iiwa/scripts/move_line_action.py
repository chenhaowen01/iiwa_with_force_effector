#!/usr/bin/env python

import rospy
import actionlib
import iiwa_msgs.msg

g_current_cartesian_pose = None

def cartesian_pose_callback(pose):
    global g_current_cartesian_pose
    # rospy.loginfo(pose)
    g_current_cartesian_pose = pose

def move_to_cartesian_pose_lin_client():
    client = actionlib.SimpleActionClient('/iiwa/action/move_to_cartesian_pose_lin', iiwa_msgs.msg.MoveToCartesianPoseAction)

    while not g_current_cartesian_pose:
        rospy.loginfo('wait for current pose!')
        rospy.sleep(1)
    
    rospy.loginfo(g_current_cartesian_pose)

    goal = iiwa_msgs.msg.MoveToCartesianPoseGoal()
    goal.cartesian_pose = g_current_cartesian_pose
    goal.cartesian_pose.poseStamped.header.seq = 0
    goal.cartesian_pose.poseStamped.header.stamp = rospy.get_rostime()
    goal.cartesian_pose.poseStamped.pose.position.x -= 0.1
    rospy.loginfo(goal)

    client.wait_for_server()
    client.send_goal_and_wait(goal)
    result = client.get_result()
    if result.success:
        rospy.loginfo('move to pose sucess!')
    else:
        rospy.loginfo('move to pose failed: {}'.format(result.error))
        return
    
    goal.cartesian_pose.poseStamped.header.seq += 1
    goal.cartesian_pose.poseStamped.header.stamp = rospy.get_rostime()
    goal.cartesian_pose.poseStamped.pose.position.x += 0.1
    client.send_goal_and_wait(goal)
    result = client.get_result()
    if result.success:
        rospy.loginfo('move to pose sucess!')
    else:
        rospy.loginfo('move to pose failed: {}'.format(result.error))

if __name__ == "__main__":
    rospy.init_node('move_to_cartesian_pose_lin_client')
    rospy.Subscriber('/iiwa/state/CartesianPose', iiwa_msgs.msg.CartesianPose, cartesian_pose_callback)
    try:
        move_to_cartesian_pose_lin_client()
    except rospy.ROSInterruptException:
        rospy.loginfo('program interrupted before completion')
