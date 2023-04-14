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
from std_msgs.msg import String

def shutdown():
    rospy.loginfo("shutting down...")


class Robot:
    def __init__(self):
        self._bridge = CvBridge()
        odom_sub =rospy.Subscriber("/odom",Odometry,self.odom_callback)
        target_sub = rospy.Subscriber("/coord_centroid",Point,self.target_callback)
        self._pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        self._diagnostic = rospy.Publisher("/nibbles_diag", String,queue_size=1)
        self._move=Twist()
        self._target_x=999.0
        self._target_y=999.0
        self._origin=(0.0, 0.0)
        self._midpoint = 320.0
        self._contact_point = 250.0
        self._block_captured = False
        self._block_found = False
        self._going_home = False
        self._started = False
        self._is_home = False
        self._staging = True
        self._is_complete = False
        self._rtb_flag = False
        self._rate = rospy.Rate(2)
        self._img = None
        self._yaw = 0.0
        self._curr_x = 0.0
        self._curr_y = 0.0
        self._curr_heading = 0.0
        self._x_offset = 0.0
        self._y_offset = 0.0
        self._adj_offset_x = 0.0
        self._adj_offset_y = 0.0
        self._last_x = -999.0
        self._last_y = -999.0
        self._stage_x = 0.20
        self._last_direction = 0.0
        self._blocks_found = 0
        self._area_size_x = 0.75
        self._area_size_y = 0.75
        self._blocks_total = 2

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
        self.set_adjust()


    def last_position(self):
        self._last_x = self._adj_offset_x
        self._last_y = self._adj_offset_y
        self._diagnostic.publish(f"X: {self._last_x} Y: {self._last_y}")
        rospy.loginfo(f"X: {self._last_x} Y: {self._last_y}")

    def position_zero(self):
        self._x_offset = self._curr_x
        self._y_offset = self._curr_y


    def set_adjust(self):
        self._adj_offset_y = self._curr_y - self._y_offset
        self._adj_offset_x = self._curr_x - self._x_offset


    def next_stage(self):
        self._stage_x = self._stage_x + 0.2


    def drive(self):
        count = 6
        while count > 0 and self._started is False:
            count = count - 1
            self._move.linear.x = 0.05
            self._move.angular.z = 0.0
            self._pub.publish(self._move)
            self._rate.sleep()
        self._started = True
        if self._blocks_found >= self._blocks_total:
            self._is_complete = True
        elif self._rtb_flag is False and self._adj_offset_x >= self._area_size_x or self._adj_offset_x <= 0.0 or self._adj_offset_y >= self._area_size_y:
            self._last_direction = self._curr_heading
            rospy.loginfo("Turning")
            self.roomba()
        elif self._block_found is False and self._block_captured is False and self._going_home is False:
            self.find_block()
            rospy.loginfo("finding block")
        elif self._block_found is True and self._block_captured is False and self._going_home is False:
            self.drive_to_object()
 #           rospy.loginfo("Driving 2 Block")
        elif self._block_found is True and self._block_captured is True and self._is_home is False:
            self.rtb()
            self._rtb_flag = True
            rospy.loginfo("RTB")
        elif self._is_home is True and self._block_found is True:
            self._blocks_found = self._blocks_found + 1
            self.back_away()
            self.face_last_position()
            self.correct(True)
            self.face_last_direction()
            self._is_home = False
            self._block_found = False
            self._block_captured = False
            self._going_home = False
            self._rtb_flag = False
            self.next_stage()
            rospy.loginfo("next target")
        else:
            self._is_complete = True
            rospy.loginfo("Done")


    def roomba(self):
        if self._last_direction == 0.0:
            direction = True
        elif abs(self._last_direction) == math.pi:
            direction = False

        self.all_stop()
        self._curr_heading = -(math.pi / 2)
        self.turn(direction,True)
        self._move.linear.x = 0.05
        self._last_y = self._curr_y
        while abs(self._curr_y - self._last_y) < 0.130:
            self._pub.publish(self._move)
            self._rate.sleep()

        self.all_stop()

        if direction is True:
            self._curr_heading = math.pi
        elif direction is False:
            self._curr_heading = 0.0

        self.turn(direction,True)
        self._last_direction = self._curr_heading
        self.all_stop()
        while self._adj_offset_x > self._area_size_x or self._adj_offset_x < 0.0:
            self._move.linear.x =0.02
            self._pub.publish(self._move)
        self.all_stop()



    def find_block(self):
        self._move.linear.x = 0.04
        self._move.angular.z = 0.0
        self._pub.publish(self._move)
        if self._target_x != 999.0:
            self._block_found = True
            self.last_position()
#        rospy.loginfo("Finding block")


    def drive_to_object(self):
#        rospy.loginfo(f"X:  {self._target_x}  Y: {self._target_y}")
        if self._target_x != 999.0:
            adj_x= self._midpoint - self._target_x
            correction = adj_x / 400
        else:
            correction = 0.0
        self._move.angular.z = correction

        if self._target_y <= self._contact_point or self._target_y == 999.0:
            self._move.linear.x =0.04
        elif self._going_home is True and self._target_y <= 240 or self._target_y ==999.0:
            self.move.linear.x = 0.04
        else:
            if self._going_home is True:
                self._is_complete = True
            self._block_captured = True
            self.all_stop()
        self._pub.publish(self._move)


    def rtb(self, home_point = False):
 #       rospy.loginfo("RTB")
        if self._curr_heading != 99.0 and self._going_home is False :
            if math.isclose(self._last_direction, math.pi, abs_tol = 0.017):
                self.turn(True)
                self._going_home = True
            else:
                self.turn()
                self._going_home = True
        else:
           rospy.loginfo("RTB else")
           self.correct()
           self._is_home = True

    def back_away(self):
        self._move.angular.z = 0.0
        self._move.linear.x = -0.05
        count = 6
        while count > 0:
            self._pub.publish(self._move)
            count = count - 1
            self._rate.sleep()
        self.all_stop()


    def face_last_direction(self):
        self._curr_heading = self._last_direction
        if self._curr_heading == 0.0:
            self.turn(False,True)
        else:
            self.turn(True,True)


    def face_last_position(self):
        x = self._last_x - self._curr_x
        y = self._last_y - self._curr_y
        self._curr_heading = (math.atan(y / x))
        rospy.loginfo(f"Heading = {self._curr_heading}")
        rospy.loginfo(f"Yaw = {self._yaw}")
        if (-math.pi / 2) < self._curr_heading < (math.pi / 2):
            self.turn(True,True)
        else:
            self.turn(False,True)


    def all_stop(self):
        self._move.angular.z = 0.0
        self._move.linear.x = 0.0
        self._pub.publish(self._move)

   # def is_complete(self):
   #     if self._going_home is True and math.isclose(self._curr_x, self._x_offset, abs_tol =0.1) and math.isclose(self._curr_y,self._y_offset , abs_tol=0.1):
   #         return True
   #     else:
   #         return False


    def correct(self,resuming = False):

        while not math.isclose( self._curr_x, self._stage_x, abs_tol = 0.02) and not math.isclose(self._curr_y, 0.02, abs_tol = 0.02):

            if resuming is False:
                self.get_heading()
            elif resuming is True:
                self.get_heading(resuming)
            if resuming:
                if math.isclose(self._adj_offset_x, self._last_x, abs_tol = 0.05) and math.isclose(self._adj_offset_y, self._last_y, abs_tol = 0.05):
                    break
            adj_z=self._curr_heading - self._yaw
            correction = adj_z / 200
            self._move.angular.z = correction
            self._move.linear.x = 0.05
            self._pub.publish(self._move)
        self.all_stop()
        self._is_home = True

    def get_heading(self, resuming = False):
        if self._staging is True and resuming is False:
            self._curr_heading = math.atan((self._adj_offset_y - 0.05) / (self._adj_offset_x - self._stage_x))
        elif self._staging is True and resuming is True:
            self._curr_heading = -math.atan((self._last_y - self._curr_y) / (self._last_x - self._curr_x))
        else:
            self._curr_heading = math.atan(self._adj_offset_y / self._adj_offset_x)

        if self._curr_heading > 0.0:
            self._curr_heading = self._curr_heading - math.pi
        else:
            self._curr_heading = self._curr_heading + math.pi


    def turn(self,clockwise=False,row_end=False):
        while(True):
            if row_end is False:
                self._move.linear.x = 0.02
                self.get_heading()
            else:
                self._move.linear.x = 0.0

            if(clockwise):
                self._move.angular.z=-0.3
            else:
                self._move.angular.z=0.3

            self._pub.publish(self._move)

            if(self._curr_heading==math.pi):
                if(math.isclose( abs(self._yaw), self._curr_heading, abs_tol = 0.0175)):
                    break
            else:
                if(math.isclose( self._yaw, self._curr_heading, abs_tol = 0.0175)):
                    break
        self.all_stop()



def main():
    rospy.init_node("path_solver",anonymous=True)
    rospy.on_shutdown(shutdown)
    robby = Robot()
    time.sleep(3)
    robby.position_zero()
    while not rospy.is_shutdown():
        robby.drive()
        if robby._is_complete:
            break


if __name__=="__main__":
    main()
