#!/usr/bin/env python
import cv2
import numpy as np
from datetime import datetime
import array
import fcntl
import os
import argparse
from utils import ArducamUtils
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager
import arducam_mipicamera as arducam
import v4l2
import time
import cv2


def align_down(size, align):
    return (size & ~((align)-1))

def align_up(size, align):
    return align_down(size + align - 1, align)

def set_controls(camera):
    try:
        print("Reset the focus...")
        camera.reset_control(v4l2.V4L2_CID_FOCUS_ABSOLUTE)
    except Exception as e:
        print(e)
        print("The camera may not support this control.")

    try:
        print("Enable Auto Exposure...")
        camera.software_auto_exposure(enable = True)
        print("Enable Auto White Balance...")
        camera.software_auto_white_balance(enable = True)
    except Exception as e:
        print(e)


if __name__ == "__main__":
    rospy.init_node("arducam_stereo_camera")
    
    try:
        bridge = CvBridge()
        cam_pub = rospy.Publisher('arducam/image_raw', Image, queue_size=1)
        camera = arducam.mipi_camera()
        print("Open camera...")
        camera.init_camera()
        print("Setting the resolution...")
        fmt = camera.set_resolution(1920, 1080)
        print("Current resolution is {}".format(fmt))
        set_controls(camera)
        while not rospy.is_shutdown(): #cv2.waitKey(10) != 27:
            frame = camera.capture(encoding = 'i420')
            encod = "bgr8" 
            height = int(align_up(fmt[1], 16))
            width = int(align_up(fmt[0], 32))
            image1 = frame.as_array.reshape(int(height * 1.5), width)
            image = cv2.cvtColor(image1, cv2.COLOR_YUV2BGR_I420)
            try:
                ros_image = bridge.cv2_to_imgmsg(image, "bgr8")
                cam_pub.publish(ros_image)
                
            except CvBridgeError, e:
                rospy.logerr("CvBridge Error: {0}".format(e))
            
            #cv2.imshow("Arducam", image)
            
    except Exception as e:
        print(e)
    # Release memory
    del frame
    camera.close_camera()
    print("Close camera...")
    
