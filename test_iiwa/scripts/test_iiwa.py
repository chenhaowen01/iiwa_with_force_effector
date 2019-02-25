#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

def cartesian_move(group, waypoints, rate):
    ready_state = RobotState()
    joint_state = JointState()
    joint_state.name = group.get_active_joints()
    joint_state.position = group.get_current_joint_values()
    ready_state.joint_state = joint_state

    # compute a path consised of straight line segments from a series of waypoints
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.0001, 0)
    print(len(plan.joint_trajectory.points))
    print('path fraction: {}'.format(fraction))

    # arg ref_state_in is the start state of robot
    new_plan = group.retime_trajectory(ready_state, plan, 0.005)
    group.execute(new_plan)
    group.stop()

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_commander', anonymous=True)

    group = moveit_commander.MoveGroupCommander('manipulator')

    group.set_max_velocity_scaling_factor(0.005)

    # home, ready are predefined pose, defined when create moveit package
    # group.set_named_target('home')
    # group.go()
    # group.stop()
    
    group.set_named_target('ready')
    group.go()
    group.stop()

    ready_pose = group.get_current_pose().pose

    ready_pose.position.z = 0.05
    group.set_pose_target(ready_pose)
    group.go()
    group.stop()

    ready_target_pose = copy.deepcopy(ready_pose)
    ready_target_pose.position.x -= 0.2

    cartesian_move(group, [ready_target_pose, ready_pose], 0.005)

if __name__ == '__main__':
    main()