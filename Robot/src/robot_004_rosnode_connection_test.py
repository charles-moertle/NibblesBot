#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

count=0
rospy.init_node("hello_node",anonymous=True)
rate=rospy.Rate(1)
pub=rospy.Publisher("/hello_node", String, queue_size=1)
while not rospy.is_shutdown():
     pub.publish(f"Hello {count}")
     count+=1
     rate.sleep()
