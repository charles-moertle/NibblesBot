#! /usr/bin/env python3

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge=CvBridge()

def shutdown():
     cv.destroyAllWindows()
     rospy.loginfo("shutting down...")

def callback(data):
     frame=bridge.imgmsg_to_cv2(data, "passthrough")
     rospy.loginfo(f"frame size: {frame.size}")
     cv.imshow('frame',frame)
     cv.waitKey(1)

def main():
     rospy.init_node("image_listener",anonymous=True)
     sub=rospy.Subscriber("/nibbles_img",Image,callback)
     rospy.on_shutdown(shutdown)
     rospy.spin()


if __name__ == '__main__':
     main()
