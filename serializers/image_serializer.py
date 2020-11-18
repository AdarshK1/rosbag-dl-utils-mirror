"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
# from cv_bridge import CvBridge
# import cv2
from sensor_msgs.msg import Image
from serializers.Base_Serializer import BaseSerializer
import sys
import numpy as np


class ImageSerializer(BaseSerializer):
    def __init__(self, topic_name, is_depth=False, channels=3, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT IMAGE SERIALIZER")
        super(ImageSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ".npy"
        self.is_depth = is_depth
        self.channels = int(channels)
        rospy.Subscriber(topic_name, Image, self.callback)

    def callback(self, img_msg):
        # read img to cv2
        if self.is_depth:
            img = np.ndarray(shape=(img_msg.height, img_msg.width, self.channels),
                                  dtype=np.float16, buffer=img_msg.data)
            img = np.float32(img)
            # print(img.shape, self.channels, self.is_depth)
        else:
            img = np.ndarray(shape=(img_msg.height, img_msg.width, self.channels),
                             dtype=np.uint8, buffer=img_msg.data)

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, np.array([img, rospy.get_time()]))
        # cv2.imwrite(file_name, np.array([img, rospy.get_time()]))

        # increment
        self.counter += 1