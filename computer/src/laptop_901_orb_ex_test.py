#! /usr/bin/env python3

import cv2 as cv
import numpy as np
import time


class CameraCV:
    def __init__(self):
        self._camera = cv.VideoCapture(0)
        self._width = int(self._camera.get(3))
        self._height = int(self._camera.get(4))

        # ORB Training images
        self._training_images = {
            "A": cv.imread("training_imgs/a_pixel.jpg", 1),
            "B": cv.imread("training_imgs/b_pixel.jpg", 1),
        }

        # Dictionary to hold the blocks letter and centroid
        self._block_locations = {}

        self._lower_blue = np.array([100, 100, 100])
        self._upper_blue = np.array([200, 200, 255])

        # Need to find exact red lower and upper hsv
        self._lower_red = np.array([100, 100, 100])
        self._upper_red = np.array([200, 200, 255])

        # Need to find exact green lower and upper hsv
        self._lower_green = np.array([100, 100, 100])
        self._upper_green = np.array([200, 200, 255])

    def get_camera_frame(self):
        success = False

        while success is not True:
            success, frame = self._camera.read()

        return frame

    def get_height(self):
        return self._height

    def get_width(self):
        return self._width

    def mask_frame(self):
        frame = self.get_camera_frame()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, self._lower_blue, self._upper_blue)
        return mask

    def get_contours(self, frame, frame_contour):
        contours, _ = cv.findContours(frame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv.contourArea(contour)

            if area > 1500:
                cv.drawContours(frame_contour, contour, -1, (255, 255, 0), 3)
                perimeter = cv.arcLength(contour, True)
                approx = cv.approxPolyDP(contour, 0.02 * perimeter, True)
                # print(len(approx))
                x, y, width, height = cv.boundingRect(approx)
                cv.rectangle(frame_contour, (x, y), (x + width, y + height), (0, 255, 0), 5)

                find_largest_match = ("NULL", 0)

                for key in self._training_images:
                    temp = self.orb_detection(key, self.get_camera_frame()[y:y + height, x:x + width])
                    # cv.imshow("Cropped image", self.get_camera_frame()[y:y+height, x:x+width])
                    # time.sleep(1)
                    if temp is None:
                        temp = 0
                    if find_largest_match[1] < temp:
                        find_largest_match = (key, temp)

                self._block_locations[find_largest_match[0]] = (x + width // 2, y + height // 2)

                # Writes the guessed letter on the frame image
                cv.putText(frame_contour, find_largest_match[0], (x + width // 2 - 10, y + height // 2 - 10), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 4)

                # Drawing centroid circles in each box
                center = (x + width // 2, y + height // 2)
                radius = 5
                cv.circle(frame_contour, center, radius, (255, 255, 255), -1)

    def boxes_with_centroid(self):
        frame = self.get_camera_frame()
        frame_contour = frame.copy()

        blur = cv.GaussianBlur(frame, (5, 5), 1)

        gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)

        canny = cv.Canny(gray, 47, 38)

        kernel = np.ones((5, 5))
        dilation = cv.dilate(canny, kernel, iterations=1)

        # Clear the dictionary so the same blocks don't get added infinitely
        self._block_locations.clear()

        self.get_contours(dilation, frame_contour)

        # Drawing a circle at the bottom middle of the image
        center = (self._width // 2, self._height)
        radius = 5
        cv.circle(frame_contour, center, radius, (0, 0, 0), -1)

        cv.imshow("Frame", frame)
        cv.imshow("Frame Contour", frame_contour)

        return self._block_locations

    def orb_detection(self, key, testing_image):
        training_image = self._training_images.get(key)

        height, width, _ = testing_image.shape
        if height < 20 and width < 20:
            return

        training_gray = cv.cvtColor(training_image, cv.COLOR_RGB2GRAY)
        testing_gray = cv.cvtColor(testing_image, cv.COLOR_RGB2GRAY)
        # print(f"This is the gray image: {type(testing_gray)}")

        # Initialize the ORB Feature detector
        #orb = cv.ORB_create(nfeatures=1000)
        #orb = cv.ORB_create(edgeThreshold=15, patchSize=31, nlevels=12, fastThreshold=15, scaleFactor=1.1, WTA_K=4,scoreType=cv.ORB_FAST_SCORE, firstLevel=0, nfeatures=200)
        orb = cv.ORB_create(edgeThreshold=15, patchSize=31, nlevels=12, fastThreshold=15, scaleFactor=1.1, WTA_K=4, scoreType=cv.ORB_HARRIS_SCORE, firstLevel=0, nfeatures=200)

        testing_image_with_features = np.copy(testing_gray)

        # Create another copy to display points only
        dots_testing = np.copy(testing_image_with_features)
        dots_training = np.copy(training_gray)

        # Extract the keypoints from both images
        train_keypoints, train_descriptor = orb.detectAndCompute(training_gray, None)
        test_keypoints, test_descriptor = orb.detectAndCompute(testing_gray, None)

        cv.drawKeypoints(testing_gray, test_keypoints, testing_image_with_features, flags=cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv.drawKeypoints(testing_gray, test_keypoints, dots_testing, flags=cv.DRAW_MATCHES_FLAGS_DRAW_OVER_OUTIMG)
        cv.drawKeypoints(training_image, train_keypoints, dots_training, flags=cv.DRAW_MATCHES_FLAGS_DRAW_OVER_OUTIMG)

        # Checks to see if the testing_frame has enough keypoints
        # Too little keypoints and things crash
        if len(test_keypoints) < 15:
            print(f"This is the number of keypoints detected: {len(test_keypoints)}")
            return

        # Initialize the BruteForce Matcher
        bf = cv.BFMatcher(cv.NORM_HAMMING2, crossCheck=True)

        # Match the feature points from both images
        matches = bf.match(train_descriptor, test_descriptor)

        cv.imshow("Testing", testing_image_with_features)
        cv.imshow("Dots Testing", dots_testing)
        cv.imshow("Dots Training", dots_training)

        # Returning the number of matches between a training image and the contour frame
        return len(matches)

    # Finds the centroid of whatever color is being masked, returns the coordinates
    def find_centroid(self):
        mask = self.mask_frame()

        m = cv.moments(mask, False)
        try:
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
        except ZeroDivisionError:
            cy, cx = self._height / 2, self._width / 2

        centroid = (cx, cy)
        return centroid

    # Counts the numer of white pixels in a masked image
    def count_white_pixels(self):
        mask = self.mask_frame()
        white_pixels = np.sum(mask == 255)
        return white_pixels

    # Function only shows
    def show_centroid(self):
        frame = self.get_camera_frame()
        mask = self.mask_frame()

        result = cv.bitwise_and(frame, frame, mask=mask)
        centroid = self.find_centroid()
        cv.circle(result, ((int(centroid[0])), int(centroid[1])), 10, (0, 0, 255), -1)

        cv.imshow("frame", frame)
        cv.imshow("mask", mask)
        cv.imshow("centroid_mask", result)

    # Call when program is ending
    def release_camera(self):
        self._camera.release()
        cv.destroyAllWindows()


def main():
    cam = CameraCV()

    while True:

        # print(f"This is the list of centroids: {cam.boxes_with_centroid()}")
        # cam.boxes_with_centroid()

        # cam.orb_detection()

        print(f"This is the dict: {cam.boxes_with_centroid()}")

        #frame = cam.get_camera_frame()
        #cv.imshow("Frame", frame)


        if cv.waitKey(1) == ord('q'):
            #cv.imwrite("b_with_camera.jpg", frame)
            cam.release_camera()
            break


if __name__ == "__main__":
    main()
