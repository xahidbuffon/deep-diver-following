#! /usr/bin/env python

"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/


Class for detecting diver (front right camera) and publish target bounding box for following
"""

# ros/python/opencv libraries and msgs
import sys
import os
import argparse
import cv2
import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from threading import Lock
from cv_bridge import CvBridge, CvBridgeError

# local libraries and msgs
from diverDetector import DiverDetection
from bboxTracker import BoxTrackerKF
from target_detection.msg import TargetObservation, TargetObservations


class FollowerPipeline:
        """ 
           Class for detecting diver (front right camera) and publish target bounding box 
        """
	def __init__(self, real_time=False):
		self.drDetect = DiverDetection()
                self.drTracker = BoxTrackerKF()
                # target BBox center
		self.target_x, self.target_y = None, None
		self.mutex = Lock()

		self.bench_test, self.publish_image = False, True
		# settings only for real-time testing
		if real_time:
			rospy.init_node('follower', anonymous=True)
			self.bridge = CvBridge()
			self.topic_right = '/camera_front_right/image_raw'
			image_right = rospy.Subscriber(self.topic_right, Image, self.imageCallBack, queue_size=3)

                        # Bbox publisher and processed output image publisher
			self.bbox_pub  = rospy.Publisher("/target/observation", TargetObservations, queue_size=3)
			self.ProcessedRaw = rospy.Publisher('/diver_follower/image', Image, queue_size=3)
			try:
				rospy.spin()
			except KeyboardInterrupt:
				print("Rospy Sping Shut down")
			
		

	def imageCallBack(self, r_im):
                """ 
                   CallBack function to get the image (from back camera) through the 
                    ros-opencv-bridge and start processing
                """
		try:
			self.original = self.bridge.imgmsg_to_cv2(r_im, "bgr8")
		except CvBridgeError as e:
			print(e)
		if self.original is None:
			print ('frame dropped, skipping tracking')
		else:
			self.ImageProcessor()



	def ImageProcessor(self):
                """ 
                   Process each frame
			> detect diver
			> prepare and publish target bounding box
                """
                # diver detection
		BBox, success_ = self.drDetect.Detect_diver(self.original)
		n_, m_, _ = self.original.shape

		if success_:
                        # state estimation using KF (observation Bbox, estimation drTracker.box)
                        box_estimated_ = self.drTracker.estimateTrackedBbox(BBox)
			if box_estimated_:
				left, right, top, bottom = self.drTracker.box
        	                # prepare and publish target bounding box
				self.mutex.acquire()
				# the bbox msg --------------------------
				obs = TargetObservation()
				obs.header.stamp = rospy.Time.now()
				mobs = TargetObservations()
				mobs.header.stamp = obs.header.stamp
				obs.target_visible = True
				obs.top_left_x = left
				obs.top_left_y = top
				obs.width = (right-left)
				obs.height = (bottom-top)
				obs.image_width = m_
				obs.image_height = n_
				obs.class_prob = 1.0
				obs.class_name = 'diver'
				mobs.observations.append(obs)
				#--------------------------
				self.mutex.release()
				self.bbox_pub.publish(mobs)

				if self.publish_image:
					print ("Diver detected >> Bbox: {0}".format(BBox))
					print ("Diver trackedd >> Bbox: {0}".format(self.drTracker.box))
        				cv2.rectangle(self.original, (left, top), (right, bottom), (0, 255, 255), 2)
	        			cv2.circle(self.original, ((left+right)/2, (top+bottom)/2), 5, (0,0,255), -1)
					msg_frame = CvBridge().cv2_to_imgmsg(self.original, encoding="bgr8")
					self.ProcessedRaw.publish(msg_frame)
		        		if self.bench_test:
						self.showFrame(self.original, 'test_viz')
			else:
				self.drTracker.init_tracker()



	##########################################################################
	###   For bench testing with dataset images ###############################
	def showFrame(self, frame, name):
		cv2.imshow(name, frame)
		cv2.waitKey(20)

	# stream images from directory Dir_
	def image_streamimg(self, Dir_):
		from eval_utils import filter_dir
		dirFiles = filter_dir(os.listdir(Dir_))
		for filename in dirFiles:
			self.original = cv2.imread(Dir_+filename)
			self.ImageProcessor()
	####################################################################################
