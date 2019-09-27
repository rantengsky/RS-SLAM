#!/usr/bin/env python
"""
Take in an image (rgb or rgb-d)
Use CNN to do semantic segmantation
Out put a cloud point with semantic color registered
\author Xuan Zhang
\date May - July 2018
"""

from __future__ import division
from __future__ import print_function

import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

from sensor_msgs.msg import PointCloud2
from color_pcl_generator import PointType, ColorPclGenerator
import message_filters
import time
import datetime
from skimage.transform import resize
import cv2

import torch
from ptsemseg.models import get_model
from ptsemseg.utils import convert_state_dict


def color_map(N=256, normalized=False):
    """
    Return Color Map in PASCAL VOC format (rgb)
    \param N (int) number of classes
    \param normalized (bool) whether colors are normalized (float 0-1)
    \return (Nx3 numpy array) a color map
    """
    def bitget(byteval, idx):
        return ((byteval & (1 << idx)) != 0)
    dtype = 'float32' if normalized else 'uint8'
    cmap = np.zeros((N, 3), dtype=dtype)
    for i in range(N):
        r = g = b = 0
        c = i
        for j in range(8):
            r = r | (bitget(c, 0) << 7-j)
            g = g | (bitget(c, 1) << 7-j)
            b = b | (bitget(c, 2) << 7-j)
            c = c >> 3
        cmap[i] = np.array([r, g, b])
    cmap = cmap/255.0 if normalized else cmap
    return cmap


def decode_segmap(temp, n_classes, cmap):
    """
    Given an image of class predictions, produce an bgr8 image with class colors
    \param temp (2d numpy int array) input image with semantic classes (as integer)
    \param n_classes (int) number of classes
    \cmap (Nx3 numpy array) input color map
    \return (numpy array bgr8) the decoded image with class colors
    """
    r = temp.copy()
    g = temp.copy()
    b = temp.copy()
    for l in range(0, n_classes):
        r[temp == l] = cmap[l, 0]
        g[temp == l] = cmap[l, 1]
        b[temp == l] = cmap[l, 2]
    bgr = np.zeros((temp.shape[0], temp.shape[1], 3))
    bgr[:, :, 0] = b
    bgr[:, :, 1] = g
    bgr[:, :, 2] = r
    return bgr.astype(np.uint8)


class SemanticCloud:
    """
    Class for ros node to take in a color image (bgr) and do semantic segmantation on it to produce an image with semantic class colors (chair, desk etc.)
    Then produce point cloud based on depth information
    CNN: PSPNet (https://arxiv.org/abs/1612.01105) (with resnet50) pretrained on ADE20K, fine tuned on SUNRGBD or not
    """
    def __init__(self, gen_pcl = True):
        """
        Constructor
        \param gen_pcl (bool) whether generate point cloud, if set to true the node will subscribe to depth image
        """
       
        # Get image size
        self.img_width, self.img_height = rospy.get_param('/camera/width'), rospy.get_param('/camera/height')
        # Set up CNN is use semantics
        #if self.point_type is not PointType.COLOR:
        print('Setting up CNN model...')
        # Set device
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        # Get dataset
        dataset = rospy.get_param('/semantic_pcl/dataset')
        # Setup model
        model_name ='pspnet'
        model_path = rospy.get_param('/semantic_pcl/model_path')
        self.n_classes = 38 # Semantic class number
        self.model = get_model(model_name, self.n_classes, version = 'sunrgbd_res50')
        state = torch.load(model_path,map_location='cuda:0')
        self.model.load_state_dict(state)
        self.cnn_input_size = (321, 321)
        self.mean = np.array([104.00699, 116.66877, 122.67892]) # Mean value of dataset
	self.model = self.model.to(self.device)
	self.model.eval()
	self.cmap = color_map(N = self.n_classes, normalized = False) # Color map for semantic classes
        # Set up ROS
        print('Setting up ROS...')
        self.bridge = CvBridge() # CvBridge to transform ROS Image message to OpenCV image
        if gen_pcl:
            self.semlabel_pub = rospy.Publisher("/semantic_image/semantic_label", Image, queue_size=1)
            self.semcolor_pub = rospy.Publisher("/semantic_image/semantic_color", Image, queue_size=1)
            self.semconfi_pub = rospy.Publisher("/semantic_image/semantic_confidence", Image, queue_size=1)
            self.image_sub = rospy.Subscriber(rospy.get_param('/semantic_pcl/color_image_topic'), Image, self.color_callback, queue_size = 1, buff_size = 30*480*640)
        print('Ready.')

    def color_callback(self, color_img_ros):
        """
        Callback function for color image, de semantic segmantation and show the decoded image. For test purpose
        \param color_img_ros (sensor_msgs.Image) input ros color image message
        """
        print('callback')
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8") # Convert ros msg to numpy array
        except CvBridgeError as e:
            print(e)
        # Do semantic segmantation
        start_time=datetime.datetime.now()
        class_probs = self.predict(color_img)
        confidence, label = class_probs.max(1)
        
        confidence, label = confidence.squeeze(0).cpu().numpy(), label.squeeze(0).cpu().numpy()
        
        label = resize(label, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
        confidence = resize(confidence, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
        
        label = label.astype(np.uint8)
        confidence = confidence.astype(np.float32)
        # Add semantic class colors
        decoded = decode_segmap(label, self.n_classes, self.cmap)
        semantic_label = self.bridge.cv2_to_imgmsg(label, encoding="mono8")
        semantic_image = self.bridge.cv2_to_imgmsg(decoded, encoding="bgr8")
        semantic_confidence = self.bridge.cv2_to_imgmsg(confidence, encoding="32FC1")
        self.semlabel_pub.publish(semantic_label)
        self.semcolor_pub.publish(semantic_image)
        self.semconfi_pub.publish(semantic_confidence)
        end_time=datetime.datetime.now()
        print(str((end_time - start_time).microseconds / 1000))
        cv2.imshow('Semantic segmantation', decoded)
        cv2.imshow('Color image', color_img)
        cv2.waitKey(3)

    def predict(self, img):
        """
        Do semantic segmantation
        \param img: (numpy array bgr8) The input cv image
        """
        img = img.copy() # Make a copy of image because the method will modify the image
        #orig_size = (img.shape[0], img.shape[1]) # Original image size
        # Prepare image: first resize to CNN input size then extract the mean value of SUNRGBD dataset. No normalization
        img = resize(img, self.cnn_input_size, mode = 'reflect', anti_aliasing=True, preserve_range = True) # Give float64
        img = img.astype(np.float32)
        img -= self.mean
        # Convert HWC -> CHW
        img = img.transpose(2, 0, 1)
        # Convert to tensor
        img = torch.tensor(img, dtype = torch.float32)
        img = img.unsqueeze(0) # Add batch dimension required by CNN
        with torch.no_grad():
            img = img.to(self.device)
            # Do inference
            since = time.time()
            outputs = self.model(img) #N,C,W,H
            # Apply softmax to obtain normalized probabilities
            outputs = torch.nn.functional.softmax(outputs, 1)
            # print('Semantic prediction output: ', outputs)
            return outputs


def main(args):
    rospy.init_node('semantic_cloud', anonymous=True)
    seg_cnn = SemanticCloud(gen_pcl = True)
    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
