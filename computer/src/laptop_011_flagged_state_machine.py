#!/usr/bin/env python3

import smach
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
    rospy.loginfo("shutting down...")
    cv.destroyAllWindows()


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
        self._midpoint = 320.0
        self._contact_point = 250.0
        
        self._block_captured = False
        self._block_found = False
        self._going_home = False
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
        # rospy.loginfo(f"X: {self._last_x} Y: {self._last_y}")

    def position_zero(self):
        self._x_offset = self._curr_x
        self._y_offset = self._curr_y

    def set_adjust(self):
        self._adj_offset_y = self._curr_y - self._y_offset
        self._adj_offset_x = self._curr_x - self._x_offset

    def next_stage(self):
        self._stage_x = self._stage_x + 0.2

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

# Global robot instance
robby = Robot()


class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['found', 'turn', 'keep_searching', 'done'],)

    def execute(self, ud):
        robby._move.linear.x = 0.05
        robby._move.angular.z = 0.0
        robby._pub.publish(robby._move)

        # Counts the number of blocks to be found on the field
	    # If it is complete, an if statement in the main while loop will break
        if robby._blocks_found >= robby._blocks_total:
            robby._is_complete = True
            next_state = 'done'
            rospy.loginfo("All blocks have been found - exiting state machine")

        # Stand in for now, used to be find_block function
        elif robby._target_x != 999.0 and robby._block_found is False:
            robby._block_found = True
            robby.last_position()

            # If we found the block, but we havent captured it and we aren't going home, then go get the block
            if robby._block_found is True and robby._block_captured is False and robby._going_home is False:
                next_state = 'found'
                rospy.loginfo("Block has been found - moving to 'ObjectFound' state")

	    # If we arent returning to the base, and if one of a couple of area bounds are crossed then ??
        elif robby._rtb_flag is False and robby._adj_offset_x >= robby._area_size_x or robby._adj_offset_x <= 0.0 or robby._adj_offset_y >= robby._area_size_y:
            robby._last_direction = robby._curr_heading
            next_state = 'turn'
            rospy.loginfo("Reached a wall - moving to 'TurnAround' state")

	    # If block not found or captured and we arent going home, keep searching
        elif robby._block_found is False and robby._block_captured is False and robby._going_home is False:
            next_state = 'keep_searching'
            rospy.loginfo("Searching for block")

	    # I guess this is just in case the first if statement fails to trip?
        else:
            self._is_complete = True
            rospy.loginfo("Done")

        return next_state


class TurnAround(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['search', 'keep_turning'],)

    def execute(self, ud):
        if robby._last_direction == 0.0:
            direction = True
        elif abs(robby._last_direction) == math.pi:
            direction = False

        robby.all_stop()
        robby._curr_heading = -(math.pi / 2)
        robby.turn(direction,True)
        robby._move.linear.x = 0.05
        robby._last_y = robby._curr_y
        while abs(robby._curr_y - robby._last_y) < 0.130:
            robby._pub.publish(robby._move)
            robby._rate.sleep()

        robby.all_stop()

        if direction is True:
            robby._curr_heading = math.pi
        elif direction is False:
            robby._curr_heading = 0.0

        robby.turn(direction,True)
        robby._last_direction = robby._curr_heading
        robby.all_stop()
        while robby._adj_offset_x > robby._area_size_x or robby._adj_offset_x < 0.0:
            robby._move.linear.x =0.02
            robby._pub.publish(robby._move)
        robby.all_stop()

        next_state = 'search'

        return next_state


class ObjectFound(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['stage', 'found_continue'],)

    def execute(self, ud):
        # rospy.loginfo(f"X:  {self._target_x}  Y: {self._target_y}")
        if robby._target_x != 999.0:
            adj_x= robby._midpoint - robby._target_x
            correction = adj_x / 400
        else:
            correction = 0.0
        robby._move.angular.z = correction

        if robby._target_y <= robby._contact_point or robby._target_y == 999.0:
            robby._move.linear.x =0.04
            next_state = 'found_continue'
            rospy.loginfo("Moving to catch block - its close")
        elif robby._going_home is True and robby._target_y <= 240 or robby._target_y ==999.0:
            robby.move.linear.x = 0.04
            next_state = 'found_continue'
            rospy.loginfo("Moving to catch block - its far")
        else:
            if robby._going_home is True:
                robby._is_complete = True
            robby._block_captured = True

            robby.all_stop()

            next_state = 'stage'
            rospy.loginfo("Block has been caught - moving to 'StagingPoint' state")

            # Originally part of the drive function - kept for now
            # elif self._block_found is True and self._block_captured is True and self._is_home is False:
            robby._rtb_flag = True
        robby._pub.publish(robby._move)

        return next_state


class StagingPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['arrange', 'to_last', 'keep_staging'],)

    def execute(self, ud):
        if robby._curr_heading != 99.0 and robby._going_home is False :
            if math.isclose(robby._last_direction, math.pi, abs_tol = 0.017):
                robby.turn(True)
                robby._going_home = True
            else:
                robby.turn()
                robby._going_home = True
            next_state = 'keep_staging'
            rospy.loginfo("Delivering block to staging area")
        else:
           robby.correct()
           robby._is_home = True
           next_state = 'to_last'
           rospy.loginfo("Block delievered to staging area - moving to 'ReturnToLast' state")
        
        return next_state


class ReturnToLast(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['find_next', 'returning'],)

    def execute(self, ud):
        returned_to_last = False

        if returned_to_last:
            next_state = 'returning'
        else:
            next_state = 'find_next'


        # If we are at home with a block, deposit it. Also reset flags and increment necessary counts.
        if robby._is_home is True and self._block_found is True:
            robby._blocks_found = self._blocks_found + 1

            robby._move.angular.z = 0.0
            robby._move.linear.x = -0.05
            count = 6
            while count > 0:
                robby._pub.publish(robby._move)
                count = count - 1
                robby._rate.sleep()

            robby.all_stop()


            robby.face_last_position()
            robby.correct(True)
            robby.face_last_direction()
            robby._is_home = False
            robby._block_found = False
            robby._block_captured = False
            robby._going_home = False
            robby._rtb_flag = False
            robby.next_stage()
            next_state = 'find_next'
            rospy.loginfo("Beginning search for next block - move to 'Search' state")

        return next_state


class Arrange(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['keep_arranging', 'done'],)

    def execute(self, ud):
        arranged = False

        if arranged:
            next_state = 'done'
        else:
            next_state = 'keep_arranging'

        return next_state


def main():
    rospy.init_node("roomba_solver",anonymous=True)
    rospy.on_shutdown(shutdown)
    time.sleep(3)
    robby.position_zero()

    sm = smach.StateMachine(outcomes=['exit'])

    with sm:
        smach.StateMachine.add('SEARCH', Search(),
                                transitions={
                                    'found' : 'OBJECTFOUND',
                                    'turn'  : 'TURNAROUND',
                                    'keep_searching' : 'SEARCH',
                                    'done'  : 'exit',
                                })

        smach.StateMachine.add('TURNAROUND', TurnAround(), 
                                transitions={
                                    'search' : 'SEARCH',
                                    'keep_turning' : 'TURNAROUND'
                                })

        smach.StateMachine.add('OBJECTFOUND', ObjectFound(), 
                                transitions={
                                    'stage' : 'STAGINGPOINT',
                                    'found_continue' : 'OBJECTFOUND'
                                })

        smach.StateMachine.add('STAGINGPOINT', StagingPoint(),
                                transitions={
                                    'arrange' : 'ARRANGE',
                                    'to_last'  : 'RETURNTOLAST',
                                    'keep_staging' : 'STAGINGPOINT'
                                })

        smach.StateMachine.add('RETURNTOLAST', ReturnToLast(), 
                                transitions={
                                    'find_next' : 'SEARCH',
                                    'returning' : 'RETURNTOLAST'
                                })

        smach.StateMachine.add('ARRANGE', Arrange(), 
                                transitions={
                                    'keep_arranging' : 'ARRANGE',
                                    'done' : 'exit'
                                })
    sm.execute()


if __name__ == "__main__":
    main()
