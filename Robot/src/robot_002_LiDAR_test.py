#!/usr/bin/env python3

import time
import rospy
from sensor_msgs.msg import LaserScan


global front
def callback(msg):
     global front
     front=msg.ranges[127]




def main():
     rospy.init_node("nibbles_laser_node",anonymous=True)
     laser =rospy.Subscriber("/scan",LaserScan,callback)

     time.sleep(3)
     global front
     front = 0.0
     while not rospy.is_shutdown():
          print(front)
          time.sleep(2)


if __name__ == "__main__":
    main()
