#! /usr/bin/env python
"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/
Any part of this repo can be used for academic and educational purposes only
"""

import cv2
# local libraries
from diverDetector import DiverDetection
from bboxTracker import BoxTrackerKF
from utils import draw_box_label


class FollowerPipeline:
        """ 
           Class for tracking a (single) diver through image sequences 
        """
	def __init__(self, real_time=False):
		self.drDetect = DiverDetection()
                self.drTracker = BoxTrackerKF()
                self.i = 610


	def ImageProcessor(self, img, vizualize=False, wait_time=1):
                """ 
                   Process each frame
			> detect diver
			> prepare and publish target bounding box
                """
                # diver detection
                img = cv2.resize(img, (400, 300))
		BBox, success_ = self.drDetect.Detect_diver(img)
		n_, m_, _ = img.shape
		if success_:
                        # state estimation using KF (observation Bbox, estimation drTracker.box)
                        box_estimated_ = self.drTracker.estimateTrackedBbox(BBox)
			if not box_estimated_:
				self.drTracker.init_tracker()
			self.annotate_and_vizualize(img, vizualize, wait_time)


	def annotate_and_vizualize(self, img, vizualize=False, wait_time=1):
                """ 
                   Annotates bounding box, label and then shows it
                """
		if vizualize:
			img = draw_box_label(img, self.drTracker.box, show_label=True)
			cv2.imshow('test', img)
			cv2.waitKey(wait_time)
                        cv2.imwrite('/home/xahid/datasets/diver_robot_test/output/'+str(self.i)+'.jpg', img)
                        self.i+=1


