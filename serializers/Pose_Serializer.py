"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from geometry_msgs.msg import PoseStamped
from serializers.Base_Serializer import BaseSerializer
import numpy as np


class PoseSerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT PoseStamped SERIALIZER")
        super(PoseSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""

        rospy.Subscriber(topic_name, PoseStamped, self.callback)

    def callback(self, pose_msg):
        pose_arr = [pose_msg.pose.position.x,
                    pose_msg.pose.position.y,
                    pose_msg.pose.position.z,
                    pose_msg.pose.orientation.x,
                    pose_msg.pose.orientation.y,
                    pose_msg.pose.orientation.z,
                    pose_msg.pose.orientation.w, rospy.get_time()]

        np_arr = np.asarray(pose_arr)

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, np_arr)

        # increment
        self.counter += 1