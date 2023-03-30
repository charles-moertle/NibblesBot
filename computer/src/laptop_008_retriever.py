#! /usr/bin/env python3


import time
import numpy as np
import rospy
import math
import cv2 as cv
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point
from queue import Queue
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def shutdown():
    cv.destroyAllWindows()
    rospy.loginfo("shutting down...")


class Robot:
    def __init__(self):
        self._bridge = CvBridge()
        odom_sub =rospy.Subscriber("/odom",Odometry,self.odom_callback)
        target_sub = rospy.Subscriber("/coord_centroid",Point,self.target_callback)
        self._pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self._move=Twist()
        self._target_x=999.0
        self._target_y=999.0
        self._origin=(0.0, 0.0)
        self._midpoint = 320
        self._contact_point = 300
        self._block_captured = False
        self._block_found = False
        self._going_home = False

        self._img = None
        self._yaw = 0.0
        self._curr_x = 0.0
        self._curr_y = 0.0
        self._curr_heading = 99.0
        self._x_offset = 0.0
        self._y_offset = 0.0
        self._adj_offset_x = 0.0
        self._adj_offset_y = 0.0


    def target_callback(self, data):
        (self._target_x, self._target_y) = data.x, data.y


    def image_callback(self, data):
        self._img = self._bridge.imgmsg_to_cv2(data, "passthrough")


    def odom_callback(self,data):
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self._yaw) = euler_from_quaternion (orientation_list)
        self._curr_x = data.pose.pose.position.x
        self._curr_y = data.pose.pose.position.y


    def position_zero(self):
        self._x_offset = self._curr_x
        self._y_offset = self._curr_y


    def set_adjust(self):
        self._adj_offset_y = self._curr_y - self._y_offset
        self._adj_offset_x = self._curr_x - self._x_offset

    def drive(self):
        if self._block_found is False and self._block_captured is False and self._going_home is False:
            self.find_block()
        elif self._block_found is True and self._block_captured is False and self._going_home is False:
            self.drive_to_object()
        elif self._block_found is True and self._block_captured is True:
            self.rtb()

    def find_block(self):
        self._move.linear.x = 0.05
        self._move.angular.z = 0.0
        self._pub.publish(self._move)
        if self._target_x != 999.0:
            self._block_found = True
        rospy.loginfo("Finding block")


    def drive_to_object(self):
        rospy.loginfo("Block found")
        if self._target_x != 999.0:
            adj_x= self._midpoint - self._target_x
            correction = adj_x / 200
        else:
            correction = 0.0
        self._move.angular.z = correction

        if self._target_y < self._contact_point or self._target_y == 999.0:
            self._move.linear.x =0.05
        else:
            self._block_captured = True
            self._move.linear.x = 0.0
            self._move.angular.z = 0.0
        self._pub.publish(self._move)


    def rtb(self):
        if self._curr_heading == 99.0:
            rospy.loginfo("RTB")
            self.get_heading()
            rospy.loginfo(f"heading = {self._curr_heading}")
        elif self._curr_heading != 99.0 and self._going_home is False :
            if math.isclose(abs(self._yaw), math.pi, abs_tol = 0.017):
                self.turn(True)
                self._going_home = True
            else:
                self.turn()
                self._going_home = True
        else:
           self.correct()


    def is_home(self):
        if self._going_home is True and math.isclose(self._curr_x, self._x_offset, abs_tol =0.1) and math.isclose(self._curr_y,self._y_offset , abs_tol=0.1):
            return True
        else:
            return False


    def correct(self):
        while not math.isclose( self._curr_x, 0.0, abs_tol= 0.1) and not math.isclose(self._curr_y, 0.0, abs_tol = 0.1):
            self.get_heading()
            adj_z=self._curr_heading - self._yaw
            correction = adj_z / 200
            self._move.angular.z = correction
            self._move.linear.x = 0.05
            self._pub.publish(self._move)
        self._move.angular.z = 0.0
        self._move.linear.x = 0.0
        self._pub.publish(self._move)

    def get_heading(self):
        self._curr_heading= math.atan(self._curr_y / self._curr_x)
        if self._curr_heading > 0.0:
            self._curr_heading = self._curr_heading - math.pi
        else:
            self._curr_heading = self._curr_heading + math.pi


    def turn(self,clockwise=False):
        while(True):
            if(clockwise):
                self._move.angular.z=-0.3
                self._move.linear.x=0.05
            else:
                self._move.angular.z=0.3
                self._move.linear.x=0.05
            self._pub.publish(self._move)
            self.get_heading()
            if(self._curr_heading==math.pi):
                if(math.isclose( abs(self._yaw), self._curr_heading, abs_tol = 0.175)):
                    break
            else:
                if(math.isclose( self._yaw, self._curr_heading, abs_tol = 0.0175)):
                    break
        self._move.linear.x = 0.0
        self._move.angular.z = 0.0
        self._pub.publish(self._move)



def main():
    rospy.init_node("path_solver",anonymous=True)
    rospy.on_shutdown(shutdown)
    robby = Robot()
    time.sleep(3)
    rate = rospy.Rate(3)
    robby.position_zero()
    while not robby.is_home():
        robby.drive()


if __name__=="__main__":
    main()
