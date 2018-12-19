#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Helper classes and functions for visualizing detection
"""

import numpy as np
import cv2


def box_iou(a, b):
    '''
    Helper funciton to calculate intersection over the union of two boxes a and b
     Bbox coordinates =>> {left, right, top, bottom}
    '''
    w_intsec = np.maximum (0, (np.minimum(a[1], b[1]) - np.maximum(a[0], b[0])))
    h_intsec = np.maximum (0, (np.minimum(a[3], b[3]) - np.maximum(a[2], b[2])))
    s_intsec = w_intsec * h_intsec
    s_a = (a[1] - a[0])*(a[3] - a[2])
    s_b = (b[1] - b[0])*(b[3] - b[2])
  
    return float(s_intsec)/(s_a + s_b -s_intsec)



def handle_bad_corners(left, right, top, bottom, im_w, im_h):
    '''
    Helper fucntion for checking if the box goes outside the image
    If so, set it to boundary 
    '''
    left = np.maximum(0, left)
    top = np.maximum(0, top)
    right = np.minimum(im_w, right)
    bottom = np.minimum(im_h, bottom)
    
    return (left, right, top, bottom)
    

    
def draw_box_label(img, bbox_cv2, bbox_class="diver", box_color=(0, 0, 255), show_label=True):
    '''
    Helper funciton for drawing the bounding boxes and the labels
    bbox_cv2 = [left, right, top, bottom]
    '''
    img_h, img_w = img.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_size = 0.5
    font_color = (0, 0, 0)
    left, right, top, bottom = handle_bad_corners(bbox_cv2[0], bbox_cv2[1], bbox_cv2[2], bbox_cv2[3], img_w, img_h)
    
    # Draw the bounding box
    cv2.rectangle(img, (left, top), (right, bottom), box_color, 4)
    
    if show_label:
        # Draw a filled box on top of the bounding box (as the background for the labels)
        left1, top1, right1, _ = handle_bad_corners(left-2, top-40, right+2, bottom, img_w, img_h)
        cv2.rectangle(img, (left1, top1), (right1, top), box_color, -1, 1)
        
        # Output the labels that show the x and y coordinates of the bounding box center.
        text_label= bbox_class
        top2 = 0 if top<25 else top-25
        cv2.putText(img, text_label, (left, top2), font, font_size, font_color, 1, cv2.LINE_AA)
        text_xy= 'x='+str((left+right)/2)+' y='+str((top+bottom)/2)
        cv2.putText(img, text_xy, (left,top2+20), font, 0.4, font_color, 1, cv2.LINE_AA)
    
    return img 



"""
vizializer that draws boxes and labels on the localized objects
inputs:
    img: image be annotated
    localized_objs: list of tupples (class id, box)  
    obj_classes: dictionary with items {class_id: class_label} 
"""
def draw_boxes_and_labels(img, localized_objs, obj_classes, box_color=(0, 255, 255)):
    img_h, img_w = img.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_size = 0.5
    font_color = (0, 0, 0)

    for (i, bbox_cv2) in localized_objs:
        left, right, top, bottom = handle_bad_corners(bbox_cv2[0], bbox_cv2[1], bbox_cv2[2], bbox_cv2[3], img_w, img_h)
        cv2.rectangle(img, (left, top), (right, bottom), box_color, 4)

        # Draw a filled box on top of the bounding box (as the background for the labels)
        left1, top1, right1, _ = handle_bad_corners(left-2, top-40, right+2, bottom, img_w, img_h)
        cv2.rectangle(img, (left1, top1), (right1, top), box_color, -1, 1)
        
        # Output the labels that show the x and y coordinates of the bounding box center.
        text_label= obj_classes[i]
        top2 = 0 if top<25 else top-25
        cv2.putText(img, text_label, (left, top2), font, font_size, font_color, 1, cv2.LINE_AA)
        text_xy= 'x='+str((left+right)/2)+' y='+str((top+bottom)/2)
        cv2.putText(img, text_xy, (left,top2+20), font, 0.4, font_color, 1, cv2.LINE_AA)


    return img



im_ext_ = ['jpg', 'jpeg', 'bmp', 'png', 'ppm', 'pgm'] 
def check_file_ext(f_name):
    global im_ext_
    for ext_ in im_ext_:
        if f_name.lower().endswith(ext_):
            return True
    return False


# resizes bounding boxes with respect to resizing images to (new_size * new_size)
def D_normalise_rect(shape_, x_, y_, w_, h_):
  scale_x = (1.0/224)*shape_[1] 
  scale_y = (1.0/224)*shape_[0]
  return int(x_*scale_x),  int(y_*scale_y), int(w_*scale_x), int(h_*scale_y)
  


# resizes bounding boxes with respect to resizing images to (new_size * new_size)
def normalise_rect(im, x_, y_, w_, h_):
  scale_x = (1.0/im.shape[1])*224
  scale_y = (1.0/im.shape[0])*224
  return int(x_*scale_x),  int(y_*scale_y), int(w_*scale_x), int(h_*scale_y)


# Numerically-stable softmax function 
# help from: http://cs231n.github.io/linear-classify/#softmax
def softmax(x):
    z = x - np.max(x)
    sm =  (np.exp(z) / np.sum(np.exp(z)))
    return sm



   
