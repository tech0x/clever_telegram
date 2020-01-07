#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import glob
import yaml
import argparse
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from rospkg import RosPack

rospy.init_node('undistortion')
camera_info = None
bridge = CvBridge()

def opencv_matrix(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    mat = np.array(mapping["data"])
    mat.resize(mapping["rows"], mapping["cols"])
    return mat

yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix)

def camera_info_callback(data):
    global camera_info
    camera_info = data

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    undistorted = get_undistorted_image(cv_image, matrix, distortions)
    image_pub.publish(bridge.cv2_to_imgmsg(undistorted, 'bgr8'))

def get_matrix_and_distortions(camera_info_path):
    skip_lines = 2
    with open(camera_info_path) as infile:
        s = infile.readline()
        if s[0]!="%":
            skip_lines = 0
    with open(camera_info_path) as infile:
        for i in range(skip_lines):
            s = infile.readline()
        camera_info = yaml.load(infile)
        matrix = camera_info['camera_matrix']
        distortions = camera_info['distortion_coefficients']
        return matrix, distortions

def get_matrix_and_distortions_ros(camera_info_topic):
    global camera_info
    camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, camera_info_callback)

    x=0
    while camera_info == None and x < 20:
        print "Wait for {} information".format(camera_info_topic)
        rospy.sleep(0.1)
        x += 1 

    camera_info_sub.unregister()
    mtx = camera_info.K
    matrix = np.array([[mtx[0], mtx[1], mtx[2]], [mtx[3], mtx[4], mtx[5]], [mtx[6], mtx[7], mtx[8]]])
    distortions = np.array(camera_info.D)
    return matrix, distortions

def get_undistorted_image(cv2_image, matrix, distortions, crop_to_roi = False):
    h, w = cv2_image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(matrix, distortions, (w, h), 1, (w, h))
    undistorted = cv2.undistort(cv2_image, matrix, distortions, None, newcameramtx)
    x, y, w, h = roi
    if crop_to_roi:
        undistorted = undistorted[y:y + h, x:x + w]
    return undistorted

if __name__ == "__main__":
    matrix, distortions = get_matrix_and_distortions_ros('/main_camera/camera_info')

    print matrix
    print distortions
    if len(distortions) == 0:
        print("Empty distorsions, node exiting")
        exit()

    image_sub = rospy.Subscriber('/main_camera/image_raw', Image, image_callback)
    image_pub = rospy.Publisher('/main_camera/undistorted', Image, queue_size=1)

    rospy.spin()
