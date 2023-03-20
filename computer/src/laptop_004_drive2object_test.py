#! /usr/bin/env python3


import rospy
import math
import numpy as np
import cv2 as cv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from queue import Queue

move=Twist()
pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
target_x=0.0
target_y=0.0


def shutdown():
     rospy.loginfo("shutting down...")

def target_callback(data):
     global target_x,target_y
     (target_x,target_y)=data.x,data.y
     rospy.loginfo(f"X: {target_x}  Y: {target_y}")

def drive():
     global target_x, target_y
     midpoint = 320
     contact_point=390
     adj_x = midpoint - target_x
     correction = adj_x / 200
     move.angular.z = correction
     rospy.loginfo("here")
     if target_y < contact_point:
          move.linear.x = 0.1
     else:
          move.linear.x = 0.0
     pub.publish(move)

def main():
     rospy.init_node("fast_af_boi",anonymous=True)
     coord_sub=rospy.Subscriber("/coord_centroid",Point,target_callback)
     rate=rospy.Rate(2)
     while not rospy.is_shutdown():
          rospy.on_shutdown(shutdown)
          drive()
          rate.sleep()
     rospy.spin()



if __name__ == '__main__':
     main()
