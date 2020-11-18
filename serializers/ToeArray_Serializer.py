"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from draper_msgs.msg import ToeArray, Toe
from geometry_msgs.msg import Point
from Base_Serializer import BaseSerializer
import numpy as np


class ToeArraySerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT ToeArray SERIALIZER")
        super(ToeArraySerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""

        rospy.Subscriber(topic_name, ToeArray, self.callback)

    def callback(self, toe_msg):
        arr = []
        for toe in toe_msg.toes:
            arr.append(toe.contact)
            arr.append(toe.sphase)
            arr.append(toe.position.x)
            arr.append(toe.position.y)
            arr.append(toe.position.z)
        arr.append(rospy.get_time())

        np_arr = np.asarray(arr)

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, np_arr)

        # increment
        self.counter += 1