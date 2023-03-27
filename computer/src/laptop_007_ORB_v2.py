#! /usr/bin/env python3

import os.path
import rospy
import math
import cv2 as cv
import numpy as np
import time
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image
from cv_bridge import Cvbridge, CvBridgeError
from queue import Queue


def shutdown():
    cv.destroyAllWindows()
    rospy.loginfo("shutting down...")



class CameraCV:
    def __init__(self):
        self._bridge = CvBridge()
        self._imgs = None
        self._width = int(640)
        self._height = int(480)

        # ORB Training images
        path = __file__.split("/")
        path = "/".join(path[:len(path) - 1])
        self._training_images = {
            "A": cv.imread(path + "/training_imgs/a_pixel.jpg", 1),
            "B": cv.imread(path + "/training_imgs/b_pixel.jpg", 1),
        }

        # Training image descriptors
        self._training_descriptors = {}

        # Dictionary to hold the blocks letter as the key and the block's centroid as the value
        self._block_locations = {}

        self._lower_black = np.array([0, 0, 0])
        self._upper_black = np.array([139, 92, 59])
        sub = rospy.Subscriber("/nibbles_img",Image,self.callback)

    def callback(self, data):
        self._imgs = self._bridge.imgmsg_to_cv2(data,"passthrough")

    def get_height(self):
        return self._height

    def get_width(self):
        return self._width

    def get_contours(self, frame, frame_contour):
        contours, _ = cv.findContours(frame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        centroid_of_squares = []

        for contour in contours:
            area = cv.contourArea(contour)

            if area > 1500:
                cv.drawContours(frame_contour, contour, -1, (255, 255, 0), 3)
                perimeter = cv.arcLength(contour, True)
                approx = cv.approxPolyDP(contour, 0.02 * perimeter, True)
                # print(len(approx))
                x, y, width, height = cv.boundingRect(approx)
                cv.rectangle(frame_contour, (x, y), (x + width, y + height), (0, 255, 0), 5)

                # List containing the bounding box information
                centroid_of_squares.append([x, y, width, height])

                # Drawing centroid circles in each box
                center = (x + width // 2, y + height // 2)
                radius = 5
                cv.circle(frame_contour, center, radius, (255, 255, 255), -1)

        return centroid_of_squares

    def boxes_with_centroid(self):
        frame = self._imgs
        frame_contour = frame.copy()

        blur = cv.GaussianBlur(frame, (5, 5), 1)

        gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)

        canny = cv.Canny(gray, 47, 38)

        kernel = np.ones((5, 5))
        dilation = cv.dilate(canny, kernel, iterations=1)

        centroid_of_squares = self.get_contours(dilation, frame_contour)

        # Drawing a circle at the bottom middle of the image
        center = (self._width // 2, self._height)
        radius = 5
        cv.circle(frame_contour, center, radius, (0, 0, 0), -1)

        cv.imshow("Frame", frame)
        cv.imshow("Frame Contour", frame_contour)

        return centroid_of_squares

    # !!! Must be called before using orb_detection !!!
    # This calculates the descriptors of the training images, so they aren't repeatedly calculated despite being
    # static images
    def find_training_descriptors(self):
        # Initialize the ORB Feature detector
        orb = cv.ORB_create(edgeThreshold=15, patchSize=25, nlevels=10, fastThreshold=15, scaleFactor=1.2, WTA_K=4,
                            scoreType=cv.ORB_HARRIS_SCORE, firstLevel=0, nfeatures=200)

        for key in self._training_images:
            # Convert the training images to gray scale
            training_gray = cv.cvtColor(self._training_images.get(key), cv.COLOR_BGR2GRAY)

            # Extract the keypoints from the training images
            _, train_descriptor = orb.detectAndCompute(training_gray, None)

            # Assign the training image descriptors to the training dictionary under the image's key
            self._training_descriptors[key] = train_descriptor

    def orb_detection(self, centroid_of_squares):
        # Initialize the ORB Feature detector
        orb = cv.ORB_create(edgeThreshold=15, patchSize=25, nlevels=10, fastThreshold=15, scaleFactor=1.2, WTA_K=4,
                            scoreType=cv.ORB_HARRIS_SCORE, firstLevel=0, nfeatures=200)

        for center in centroid_of_squares:
            x = center[0]
            y = center[1]
            width = center[2]
            height = center[3]

            # Check if image is too small
            if height < 20 and width < 20:
                continue

            # Initialize the BruteForce Matcher
            bf = cv.BFMatcher(cv.NORM_HAMMING2, crossCheck=True)

            # Clear the dictionary so the same blocks don't get added infinitely
            self._block_locations.clear()

            # Setup variables
            number_of_loops = 10
            find_largest_match = ("NULL", 0)
            keep_track_of_guesses = {}

            for i in range(number_of_loops):
                for key in self._training_images:
                    # Crops the image to be tested from the main camera frame
                    testing_image = self.get_camera_frame()[y:y + height, x:x + width]

                    # Converts the image to be tested to gray scale
                    testing_gray = cv.cvtColor(testing_image, cv.COLOR_BGR2GRAY)

                    # Extract the keypoints and descriptors from testing image
                    test_keypoints, test_descriptor = orb.detectAndCompute(testing_gray, None)
                    train_descriptor = self._training_descriptors[key]

                    # cv.drawKeypoints(testing_gray, test_keypoints, dots_testing, flags=cv.DRAW_MATCHES_FLAGS_DRAW_OVER_OUTIMG)

                    # Checks to see if the testing_frame has enough keypoints
                    # Too little keypoints and things crash
                    if len(test_keypoints) < 15:
                        # print(f"This is the number of keypoints detected: {len(test_keypoints)}")
                        continue

                    # Match the feature points from training and testing images
                    matches = bf.match(train_descriptor, test_descriptor)

                    number_of_matches = len(matches)

                    if find_largest_match[1] < number_of_matches:
                        find_largest_match = (key, number_of_matches)

                key = find_largest_match[0]

                if key in keep_track_of_guesses:
                    keep_track_of_guesses[key] = keep_track_of_guesses[key] + 1
                    # print(f"This is the dict: {keep_track_of_guesses}")
                else:
                    keep_track_of_guesses[key] = 1
                    # print(f"This is the dict: {keep_track_of_guesses}")

            largest_key = max(keep_track_of_guesses, key=keep_track_of_guesses.get)
            number_of_guesses = keep_track_of_guesses[largest_key]
            # print(f"This is largest key: {largest_key}")

            self._block_locations[find_largest_match[0]] = [(x + width // 2, y + height // 2),
                                                            str((number_of_guesses / number_of_loops) * 100) + "%"]

            # Writes the guessed letter on the frame image
            # cv.putText(frame_contour, find_largest_match[0], (x + width // 2 - 10, y + height // 2 - 10),
            #           cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 4)

        # Returns the estimated block locations
        return self._block_locations

    # Function to mask the camera frame depending on the upper and lower hsv values passed in
    def mask_frame(self, lower_color = None, upper_color = None):
        if lower_color is None or upper_color is None:
            lower_color = self._lower_black
            upper_color = self._upper_black

        frame = self.get_camera_frame()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lower_color, upper_color)
        return mask

    # Counts the numer of white pixels in a masked image
    def count_white_pixels(self, lower_color = None, upper_color = None):
        if lower_color is None or upper_color is None:
            lower_color = self._lower_black
            upper_color = self._upper_black

        mask = self.mask_frame(lower_color, upper_color)
        white_pixels = np.sum(mask == 255)
        return white_pixels

    # !!! Call when program is ending !!!
    def release_camera(self):
        self._camera.release()
        cv.destroyAllWindows()


def main():
    rospy.init_node("Block-id_node",anonymous=True)
    rospy.on_shutdown(shutdown)
    time.sleep(3)
    cam = CameraCV()
    rate=rospy.Rate(4)
    while not rospy.is_shutdown():
        cam.boxes_with_centroid()
        rate.sleep()


if __name__ == "__main__":
    main()
