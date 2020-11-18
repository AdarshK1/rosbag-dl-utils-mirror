"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import os
import rospy

class BaseSerializer:
    def __init__(self, topic_name='', skip_frame=1, directory_name='./', bag_file=''):
        self.dir_name = directory_name
        self.counter = 0
        self.skip_frame = skip_frame

        self.filename_base = self.dir_name + bag_file.split("/")[-1][:-4] + "/" + \
                             topic_name[1:].replace("/", "_")

        os.makedirs(self.filename_base, exist_ok=True)

        self.filename_base += "/{:08d}"

    def callback(self, mesh_msg):
        pass
