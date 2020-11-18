"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d
import numpy as np
from serializers.Base_Serializer import BaseSerializer


class PCLSerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT PCL SERIALIZER")
        super(PCLSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ".pcd"

        rospy.Subscriber(topic_name, PointCloud2, self.callback)

    def callback(self, pcl_msg):
        # initialize
        source_np = []
        source_pcl = open3d.geometry.PointCloud()

        # create numpy array for each point
        for p in pc2.read_points(pcl_msg, field_names=("x", "y", "z"), skip_nans=True):
            source_np.append([p[0], p[1], p[2]])

        # assign numpy_array to Open3D
        source_np = np.array(source_np)
        source_pcl.points = open3d.utility.Vector3dVector(source_np)

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        open3d.io.write_point_cloud(file_name, source_pcl)

        # increment
        self.counter += 1

