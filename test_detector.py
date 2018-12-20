#!/usr/bin/env python
"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/
Any part of this repo can be used for academic and educational purposes only
"""

import cv2
import os
import argparse

# local libraries
from libs.diverDetector import DiverDetection
from libs.utils import draw_boxes_and_labels, check_file_ext


""" for testing detections on a set of images
    detected object classes: {diver, ROV}
"""



im_dir = 'test_data/multi/'
IMAGE_PATHS = [os.path.join(im_dir, f) for f in os.listdir(im_dir) if check_file_ext(f)]

obj_classes = {1: 'Diver', 2: 'ROV'}
drDetect = DiverDetection()


for im_file in IMAGE_PATHS:
    print ("Testing {0}".format(im_file))
    frame = cv2.imread(im_file)
    localized_objs = drDetect.Detect_multi_objs(frame) 
 
    if len(localized_objs)>0:
        frame = draw_boxes_and_labels(frame, localized_objs, obj_classes)

    cv2.imshow("Annotated Output", frame)
    cv2.waitKey(1500) 













