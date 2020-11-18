"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from geometry_msgs.msg import Twist
from serializers.Base_Serializer import BaseSerializer
import numpy as np


class TwistSerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT Twist SERIALIZER")
        super(TwistSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""

        rospy.Subscriber(topic_name, Twist, self.callback)

    def callback(self, twist_msg):
        pose_arr = [twist_msg.linear.x,
                    twist_msg.linear.y,
                    twist_msg.linear.z,
                    twist_msg.angular.x,
                    twist_msg.angular.y,
                    twist_msg.angular.z,
                    rospy.get_time()]

        np_arr = np.asarray(pose_arr)

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, np_arr)

        # increment
        self.counter += 1