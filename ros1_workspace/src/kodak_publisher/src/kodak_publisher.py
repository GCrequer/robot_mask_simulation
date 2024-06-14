#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def main():
    rospy.init_node('kodak_publisher')
    pub = rospy.Publisher('/kodak/image_equi', Image, queue_size=10)
    cap = cv2.VideoCapture('/dev/video0')
    bridge = CvBridge()
    rate = rospy.Rate(30)

    roi_x, roi_y = 14, 120
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            roi_width = frame.shape[1] - roi_x
            roi_height = frame.shape[0] - 2 * roi_y
            frame = frame[roi_y:roi_height, roi_x:roi_width]
            frame = cv2.resize(frame, (2048, 1024), interpolation=cv2.INTER_CUBIC)
            msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()