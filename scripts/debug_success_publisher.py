#!/usr/bin/env python2

#This next one for python2
##!/usr/bin/env python2


import rospy
from std_msgs.msg import Bool


def test_pub():
    rospy.init_node("success_node", anonymous=True)

    camera_pub = rospy.Publisher('success', Bool, latch=True, queue_size=10)
    rate = rospy.Rate(0.5)
    flag = True

    while not rospy.is_shutdown():
        flag = not flag
        camera_pub.publish(flag)
        rate.sleep()


if __name__ == '__main__':
    try:
        test_pub()
    except rospy.ROSInterruptException:
        pass
