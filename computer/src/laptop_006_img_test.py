#! /usr/bin/env python3

import os
import cv2 as cv
import numpy as np

img = cv.imread("training_imgs/a_pixel.jpg", 1)
cv.imshow("frame", img)
cv.waitKey()
