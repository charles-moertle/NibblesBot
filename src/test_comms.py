#! /usr/bin/env python3

import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def main():
     move = Twist()
     rospy.init_node("nibbles_listener_node",anonymous=True)
     pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
     rate=rospy.Rate(1)
     time.sleep(2)
     while not rospy.is_shutdown():
          pass
     move.linear.x=0.0
     move.angular.z=0.0
     pub.publish(move)
