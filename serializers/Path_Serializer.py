"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from serializers.Base_Serializer import BaseSerializer
import numpy as np


class PathSerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT Path SERIALIZER")
        super(PathSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""

        rospy.Subscriber(topic_name, Path, self.callback)

    def callback(self, path_msg):
        poses = []
        for pose in path_msg.poses:
            pose_arr = [pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z,
                        pose.pose.orientation.x,
                        pose.pose.orientation.y,
                        pose.pose.orientation.z,
                        pose.pose.orientation.w]
            poses.append(pose_arr)

        poses.append(rospy.get_time())
        np_arr = np.asarray(poses)

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, np_arr)

        # increment
        self.counter += 1