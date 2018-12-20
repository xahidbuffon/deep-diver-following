#!/usr/bin/env python
"""
Maintainer: Jahid (email: islam034@umn.edu)
Interactive Robotics and Vision Lab
http://irvlab.cs.umn.edu/
Any part of this repo can be used for academic and educational purposes only
"""

import cv2
import os
import glob
import argparse
# local libraries
from libs.trackerPipeline import FollowerPipeline


if __name__ == '__main__':
    """ for testing (a single) diver tracking 
            > use argument --test_vid to test video or sequence of images
        other arguments:
            --im_dir >> path of image folder
            --vid    >> path of the test video file
            --image_ext >> image extension
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--im_dir', required=False, dest='im_dir', type=str, default='test_data/im1/', help='Folder containing images')
    parser.add_argument('--vid', required=False, dest='vid', type=str, default='test_data/test1.avi', help='Video file')
    parser.add_argument('--test_vid', required=False, dest='test_vid', type=bool, default=False, help='Test video or images')
    parser.add_argument('--image_ext', type=str, default='*.jpg')
    args = parser.parse_args()

    follower = FollowerPipeline()
    if not args.test_vid:
        # test a sequence of images
        IMAGE_PATHS = glob.glob(os.path.join(args.im_dir, args.image_ext))
        IMAGE_PATHS.sort(key=lambda f: int(filter(str.isdigit, f)))
        for im_file in IMAGE_PATHS:
            frame = cv2.imread(im_file)
            follower.ImageProcessor(frame, vizualize=True, wait_time=1)           
    else:
        # test on a video file 
        counter=0   
        cap = cv2.VideoCapture(args.vid)
        while(cap.isOpened()):
            ret, frame = cap.read()
            if frame is not None:
                follower.ImageProcessor(frame, vizualize=True, wait_time=1)
            else:
                counter += 1
                if counter > 10: break

        cap.release()







