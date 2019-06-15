#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Int64


def send_force():
    pub = rospy.Publisher('motor_torque_cmd', Int64, queue_size=1)
    rospy.init_node('force_sender')
    rate = rospy.Rate(50)
    f = 0
    while not rospy.is_shutdown():
        rospy.get_rostime()
        f = 500 + 500 * math.sin(2 * math.pi * 1 * rospy.get_time())
        rospy.loginfo(f)
        pub.publish(int(f))
        rate.sleep()

if __name__ == "__main__":
    try:
        send_force()
    except rospy.ROSInterruptException:
        pass