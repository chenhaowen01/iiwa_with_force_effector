#!/usr/bin/env python

import rospy
from iiwa_msgs.msg import JointQuantity, JointPosition

ready_position = JointQuantity(a1=0, a2=1.0472, a3=0, a4=-1.0472, a5=0, a6=-0.5236, a7=0)


def main():
    rospy.init_node('move_to_ready', anonymous=True)
    joint_position_publisher = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=64)
    ready_joint_position = JointPosition()
    ready_joint_position.position = ready_position
    rospy.loginfo(ready_joint_position)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rospy.loginfo(ready_joint_position)
        ready_joint_position.header.seq += 1
        ready_joint_position.header.stamp = rospy.get_rostime()
        joint_position_publisher.publish(ready_joint_position)
        rate.sleep()

if __name__ == "__main__":
    main()