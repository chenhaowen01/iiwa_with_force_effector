#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Int64
from MMRCS1 import MMRCS1

g_motor = None
g_position = 0
g_velocity = 0
g_torque = 0

def position_cmd_callback(position):
    global g_position
    if position.data != g_position:
        g_position = position.data
        rospy.loginfo('sending position: {}'.format(g_position))
        g_motor.sent_position(g_position)

def velocity_cmd_callback(velocity):
    global g_velocity
    if velocity.data != g_velocity:
        g_velocity = velocity.data
        rospy.loginfo('sending velocity: {}'.format(g_velocity))
        g_motor.sent_velocity(g_velocity)

def torque_cmd_callback(torque):
    global g_torque
    if torque.data != g_torque:
        g_torque = torque.data
        rospy.loginfo('sending torque: {}'.format(g_torque))
        g_motor.sent_torque(g_torque)

def main():
    global g_motor

    rospy.init_node('canopen_motor_node')
    rospy.loginfo('starting canopen motor node...')

    eds_file = rospy.get_param('eds_file', './copley.eds')
    motor_id = rospy.get_param('motor_id', 1)
    motor_mode = rospy.get_param('motor_mode', 4)

    g_motor = MMRCS1(motor_id, eds_file)
    g_motor.start()
    g_motor.set_mode(4)
    g_motor.sent_torque(0)
    rospy.loginfo('canopen motor inited!')

    rospy.Subscriber('motor_position_cmd', Int64, position_cmd_callback)
    rospy.Subscriber('motor_velocity_cmd', Int64, velocity_cmd_callback)
    rospy.Subscriber('motor_torque_cmd', Int64, torque_cmd_callback)

    rospy.spin()

    rospy.loginfo('canopen motor node exiting...')
    g_motor.sent_torque(0)
    g_motor.stop()

if __name__ == "__main__":
    main()