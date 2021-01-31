"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from geometry_msgs.msg import Twist
from serializers.Base_Serializer import BaseSerializer
import numpy as np

from tf2_ros import transform_listener, buffer


class TwistSerializer(BaseSerializer):
    def __init__(self, topic_name, tf_pairs, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT TF SERIALIZER")
        super(TwistSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""
        self.tf_pairs = tf_pairs

        self.tf_buffer = buffer.Buffer()
        self.tf_listener = transform_listener.TransformListener(self.tf_buffer)

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