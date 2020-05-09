#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospkg

ROS_PACK = rospkg.RosPack()
PACK_PATH = ROS_PACK.get_path("ros_theta_s")


class ThetaConversion():

    def __init__(self, w, h):
        self.cols = 0
        self.rows = 0
        self.shift = 0
        self.cols = h
        self.rows = w

        im_x = cv2.imread(PACK_PATH + "/patterns/xmap.pgm", 2)
        im_y = cv2.imread(PACK_PATH + "/patterns/ymap.pgm", 2)  
        self.map_x = np.asarray(im_x, dtype='float32')
        self.map_y = np.asarray(im_y, dtype='float32')

    def do_conversion(self, mat):
        conv_img = self.equirectangularConversion(mat)
        return conv_img

    def equirectangularConversion(self, mat):
        mat_buf = np.zeros((1280, 720, 1), dtype=mat.dtype)
        mat_buf = cv2.remap(mat, self.map_x, self.map_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)

        return mat_buf


if __name__ == '__main__':
    rospy.init_node('theta_converter')
    camera = rospy.get_param("~/camera", default="/dev/video2")
    camera_topic = rospy.get_param("~/camera_topic", default="/theta_camera/image_raw")
    img_pub = rospy.Publisher(camera_topic, Image, queue_size=1)
    bridge = CvBridge()
    try:
        cap = cv2.VideoCapture(camera)
        ret, frame = cap.read()
        hight = frame.shape[0]
        width = frame.shape[1]
        # out = cv2.VideoWriter('/home/andrey/outpy.mp4',0x7634706d, 15, (width, hight))
        converter = ThetaConversion(width, hight)
        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = cap.read()

            mat = np.asarray(frame)
            
            conv_img = converter.do_conversion(mat)
            image_message = bridge.cv2_to_imgmsg(conv_img, "bgr8")
            img_pub.publish(image_message)
            # out.write(conv_img)

            # # Display the resulting frame
            # cv2.imshow('frame',conv_img)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
        # rospy.spin()
        # cap.release()
    except:
        rospy.logerr("No such camera")
        # rospy.spin()
        cap.release()
        
    
    # When everything done, release the capture
    
    # 
    # cv2.destroyAllWindows()
