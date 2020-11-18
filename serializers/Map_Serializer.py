"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from Base_Serializer import BaseSerializer
import numpy as np


class MapSerializer(BaseSerializer):
    def __init__(self, topic_name, skip_frame=1, directory_name='./', bag_file=''):
        rospy.logwarn("INIT Map/OccupancyGrid SERIALIZER")
        super(MapSerializer, self).__init__(topic_name, skip_frame, directory_name, bag_file)

        self.file_ext = ""

        rospy.Subscriber(topic_name, OccupancyGrid, self.callback)

    def callback(self, map_msg):
        data = list(map_msg.data)
        data.append(map_msg.info.origin.position.x)
        data.append(map_msg.info.origin.position.y)
        data.append(map_msg.info.origin.position.z)
        data.append(map_msg.info.origin.orientation.x)
        data.append(map_msg.info.origin.orientation.y)
        data.append(map_msg.info.origin.orientation.z)
        data.append(map_msg.info.origin.orientation.w)
        data.append(map_msg.info.height)
        data.append(map_msg.info.resolution)
        data.append(map_msg.info.width)
        data.append(rospy.get_time())
        np_arr = np.asarray(data)

        # save
        file_name = (self.filename_base + self.file_ext).format(self.counter)
        np.save(file_name, np_arr)

        # increment
        self.counter += 1
