"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from sensor_msgs.msg import CameraInfo
from Base_Serializer import BaseSerializer
import numpy as np


class CamInfoSerializer(BaseSerializer):
    def __init__(self, topic_name, only_once=False, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT CamInfo SERIALIZER")
        super(CamInfoSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""

        self.only_once = only_once

        self.sub = rospy.Subscriber(topic_name, CameraInfo, self.callback)

    def callback(self, info_msg):
        serializable_caminfo = dict()
        serializable_caminfo['height'] = info_msg.height
        serializable_caminfo['width'] = info_msg.width
        serializable_caminfo['K'] = np.asarray(info_msg.K)
        serializable_caminfo['D'] = np.asarray(info_msg.D)
        serializable_caminfo['R'] = np.asarray(info_msg.R)
        serializable_caminfo['P'] = np.asarray(info_msg.P)

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, serializable_caminfo)

        # increment
        self.counter += 1

        # if we only want one data point
        if self.only_once:
            self.sub.unregister()
