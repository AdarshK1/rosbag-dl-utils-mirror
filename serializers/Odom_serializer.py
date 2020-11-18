"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from nav_msgs.msg import Odometry
from serializers.Base_Serializer import BaseSerializer
import numpy as np


class OdomSerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT Odometry SERIALIZER")
        super(OdomSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""

        rospy.Subscriber(topic_name, Odometry, self.callback)

    def callback(self, odom_msg):
        # read img to cv2
        pose_arr = [odom_msg.pose.pose.position.x,
                    odom_msg.pose.pose.position.y,
                    odom_msg.pose.pose.position.z,
                    odom_msg.pose.pose.orientation.x,
                    odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z,
                    odom_msg.pose.pose.orientation.w,
                    odom_msg.twist.twist.linear.x,
                    odom_msg.twist.twist.linear.y,
                    odom_msg.twist.twist.linear.z,
                    odom_msg.twist.twist.angular.x,
                    odom_msg.twist.twist.angular.y,
                    odom_msg.twist.twist.angular.z,
                    rospy.get_time()]

        np_arr = np.asarray(pose_arr)

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, np_arr)

        # increment
        self.counter += 1