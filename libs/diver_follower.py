#! /usr/bin/env python
"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/
Any part of this repo can be used for academic and educational purposes only
"""
# ros/python/opencv libraries and msgs
import sys
import os
import cv2

# local libraries and msgs
from diverDetector import DiverDetection
from bboxTracker import BoxTrackerKF
from utils import draw_box_label

class FollowerPipeline:
        """ 
           Class for detecting diver (front right camera) and publish target bounding box 
        """
	def __init__(self, real_time=False):
		self.drDetect = DiverDetection()
                self.drTracker = BoxTrackerKF()


	def ImageProcessor(self, img, vizualize=True, wait_time=1):
                """ 
                   Process each frame
			> detect diver
			> prepare and publish target bounding box
                """
                # diver detection
		BBox, success_ = self.drDetect.Detect_diver(img)
		n_, m_, _ = img.shape
		if success_:
                        # state estimation using KF (observation Bbox, estimation drTracker.box)
                        box_estimated_ = self.drTracker.estimateTrackedBbox(BBox)
			if not box_estimated_:
				self.drTracker.init_tracker()
			else:
				self.annotate_and_vizualize(img, vizualize, wait_time)



	def annotate_and_vizualize(self, img, vizualize=False, wait_time=1):
		if vizualize:
			img = draw_box_label(img, self.drTracker.box, show_label=True)
			cv2.imshow('test', img)
			cv2.waitKey(wait_time)
