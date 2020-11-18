"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from serializers.Base_Serializer import BaseSerializer
import numpy as np


class OccupancyGridSerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT OccupancyGrid SERIALIZER")
        super(OccupancyGridSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""

        rospy.Subscriber(topic_name, OccupancyGrid, self.callback)

    def callback(self, occ_grid):
        meta_data = [occ_grid.info.resolution,
                     occ_grid.info.height,
                     occ_grid.info.width,
                     occ_grid.info.origin.position.x,
                     occ_grid.info.origin.position.y,
                     occ_grid.info.origin.angular.z]

        meta_data = np.asarray(meta_data)

        np_arr = np.asarray(occ_grid.data)
        final_arr = np.asarray([np_arr,meta_data, rospy.get_time()])
        # print(rospy.get_time())

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, final_arr)

        # increment
        self.counter += 1