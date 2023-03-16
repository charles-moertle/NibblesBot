#! /usr/bin/env python3

import rospy
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

frameWidth=640
frameHeight=480

def nothing(x):
    pass

cv.namedWindow("Thresholds for Canny")
cv.resizeWindow("Thresholds for Canny", 640, 240)
cv.createTrackbar("Threshold1", "Thresholds for Canny", 47, 255, nothing)
cv.createTrackbar("Threshold2", "Thresholds for Canny", 38, 255, nothing)
bridge=CvBridge()

def shutdown():
     cv.destroyAllWindows()
     rospy.loginfo("shutting down...")

def callback(data):
     global frame
     frame=bridge.imgmsg_to_cv2(data, "passthrough")
     rospy.loginfo(f"frame size: {frame.size}")
     cv.imshow('frame',frame)
     frameContour = frame.copy()

     blur = cv.GaussianBlur(frame, (5, 5), 1)

     gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)

     thresh1 = cv.getTrackbarPos("Threshold1", "Thresholds for Canny")
     thresh2 = cv.getTrackbarPos("Threshold2", "Thresholds for Canny")
     canny = cv.Canny(gray, thresh1, thresh2)

     kernel = np.ones((5, 5))
     dilation = cv.dilate(canny, kernel, iterations=1)

     getContours(dilation, frameContour)

     center = (frameWidth // 2, frameHeight)
     radius = 5
     cv.circle(frameContour, center, radius, (0, 0, 0), -1)

     frame_h = cv.hconcat([frame, frameContour])
     cv.imshow("Canny", canny)
     cv.imshow("Frame and Contour", frame_h)

     cv.waitKey(1)



def getContours(frame, frameContour):
    contours, _ = cv.findContours(frame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv.contourArea(contour)

        if area > 1500:
            cv.drawContours(frameContour, contour, -1, (255, 255, 0), 3)
            perimeter = cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, 0.02 * perimeter, True)
            print(len(approx))
            x, y, width, height = cv.boundingRect(approx)
            cv.rectangle(frameContour, (x, y), (x + width, y + height), (0, 255, 0), 5)
            center = (x + width // 2, y + height // 2)
            radius = 5
            cv.circle(frameContour, center, radius, (255, 255, 255), -1)



def main():
     rospy.init_node("image_listener",anonymous=True)
     sub=rospy.Subscriber("/nibbles_img",Image,callback)
     rospy.on_shutdown(shutdown)
     rospy.spin()

if __name__ == '__main__':
     main()
