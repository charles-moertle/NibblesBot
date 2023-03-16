#! /usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image



def shutdown():
     camera.release()
     rospy.loginfo("Shutting down...")


camera = cv.VideoCapture(0)
rospy.init_node("Nibbles_Image_node",anonymous=True)
pub=rospy.Publisher("/nibbles_img",Image,queue_size=1)
rospy.on_shutdown(shutdown)
rate=rospy.Rate(5)
bridge = CvBridge()
camera.set(3,640)
camera.set(4,480)

while not rospy.is_shutdown():
    rospy.loginfo("in loop")
    success, frame = camera.read()
    if success:
         image_message = bridge.cv2_to_imgmsg(frame, "passthrough")
         pub.publish(image_message)
    rate.sleep()

