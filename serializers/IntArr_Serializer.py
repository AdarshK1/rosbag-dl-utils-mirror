"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from std_msgs.msg import Int16MultiArray
from serializers.Base_Serializer import BaseSerializer
import numpy as np


class IntArrSerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT IntArray SERIALIZER")
        super(IntArrSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""

        rospy.Subscriber(topic_name, Int16MultiArray, self.callback)

    def callback(self, int_msg):
        np_arr = np.asarray(int_msg.data)
        np_arr = np.append(np_arr, rospy.get_time())
        # print(rospy.get_time())

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, np_arr)

        # increment
        self.counter += 1