"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from Base_Serializer import BaseSerializer
import sys
import numpy as np


class ImageORBSerializer(BaseSerializer):
    def __init__(self, topic_name, is_depth=False, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT IMAGE ORB SERIALIZER")
        super(ImageORBSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ".png"
        self.orb_filename_base = self.dir_name + bag_file.split("/")[-1][:-4] + "/" + \
                                 topic_name[1:].replace("/", "_") + "_orb/{:08d}"
        self.orb_file_ext = ""

        self.is_depth = is_depth
        rospy.Subscriber(topic_name, Image, self.callback)

        self.orb_descriptor = cv2.ORB()

    def callback(self, img_msg):
        # read img to cv2
        if self.is_depth:
            img = np.ndarray(shape=(img_msg.height, img_msg.width, 1),
                                  dtype=np.float16, buffer=img_msg.data)
            img = np.float32(img)
        else:
            img = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                             dtype=np.uint8, buffer=img_msg.data)

        # compute features
        kp = self.orb.detect(img, None)
        kp, des = self.orb.compute(img, kp)
        # serialize features
        np.save((self.orb_filename_base + self.orb_file_ext).format(self.counter),
                {'kp': kp, 'des': des})

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        cv2.imwrite(file_name, np.array(img))

        # increment
        self.counter += 1