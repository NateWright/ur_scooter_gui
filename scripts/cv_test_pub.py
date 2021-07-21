#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def test_pub():
    rospy.init_node("camera_pub", anonymous=True)

    cv_image = cv2.imread("test_image.png")

    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

    camera_pub = rospy.Publisher('image_raw', Image, queue_size=10)
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        camera_pub.publish(image_message)
        rate.sleep()


if __name__ == '__main__':
    try:
        test_pub()
    except rospy.ROSInterruptException:
        pass
