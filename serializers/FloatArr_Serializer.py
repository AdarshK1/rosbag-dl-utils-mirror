"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from std_msgs.msg import Float32MultiArray
from Base_Serializer import BaseSerializer
import numpy as np


class FloatArrSerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT FloatArray SERIALIZER")
        super(FloatArrSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""

        rospy.Subscriber(topic_name, Float32MultiArray, self.callback)

    def callback(self, float_msg):
        # read img to cv2
        np_arr = np.asarray(float_msg.data)
        np_arr = np.append(np_arr, rospy.get_time())
        # print(rospy.get_time())

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, np_arr)

        # increment
        self.counter += 1