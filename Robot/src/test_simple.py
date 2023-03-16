#!/usr/bin/env python3

import time
import rospy
import math
import numpy as np
import
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

global yaw
global heading


move = Twist()
pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)


def odom_callback(data):
    global yaw
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion (orientation_list)

def turn(target,clockwise=False):
    global yaw
    while(True):
        if(clockwise):
            move.angular.z=-0.4
        else:
            move.angular.z=0.4
        pub.publish(move)
        if(target==math.pi):
            if(math.isclose(abs(yaw),target,abs_tol=0.175)):
                break
        else:
            if(math.isclose(yaw,target,abs_tol=0.0175)):
                break
    all_stop()
    global heading
    heading=target

def all_stop():
    move.linear.x=0
    move.angular.z=0
    pub.publish(move)
    time.sleep(1)

def go_straight():
    move.linear.x=0.4
    pub.publish(move)

def drive():
    go_straight()
    time.sleep(2)
    all_stop()




def main():
    rospy.init_node("cam_examples",anonymous=True)
    odom= rospy.Subscriber('/odom',Odometry,odom_callback)
    time.sleep(5)
    global heading
    heading=0.0
    global yaw
    yaw=0.0
    drive()




if __name__ == '__main__':
    main()
