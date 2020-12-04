"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import argparse
import yaml
import subprocess
import time

from serializers.Base_Serializer import BaseSerializer

from serializers.image_serializer import ImageSerializer
# from serializers.PCL_serializer import PCLSerializer
# from serializers.Mesh_serializer import MeshSerializer
from serializers.Odom_serializer import OdomSerializer

from serializers.Pose_Serializer import PoseSerializer
# from serializers.orb_image_serializer import ImageORBSerializer
from serializers.FloatArr_Serializer import FloatArrSerializer
from serializers.IntArr_Serializer import IntArrSerializer
from serializers.CamInfo_Serializer import CamInfoSerializer
from serializers.Twist_Serializer import TwistSerializer
from serializers.Map_Serializer import MapSerializer
from serializers.Path_Serializer import PathSerializer
from serializers.ToeArray_Serializer import ToeArraySerializer

import rospy
from typing import List
import os

def process_yaml(filename, output_dir_name, bag_name) -> List[BaseSerializer]:
    parsed_yaml = dict()
    serializer_list = list()

    if output_dir_name[-1] != '/':
        output_dir_name += '/'

    output_dir_name += bag_name.split(".")[0] + '/'

    with open(filename, 'r') as stream:
        try:
            parsed_yaml = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            rospy.logerr_once(exc.__str__())

        for item in parsed_yaml:
            name = list(item.keys())[0]
            type = item[name]["type"]
            topic_name = item[name]["topic_name"]

            ser = None

            if type == "Mesh":
                ser = MeshSerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))

            elif type == "PointCloud":
                ser = PCLSerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))

            elif type == "Image":
                is_depth = bool(item[name]['is_depth'])
                channels = int(item[name]['channels'])
                ser = ImageSerializer(topic_name, is_depth, channels, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))

            elif type == "ORB_Image":
                is_depth = bool(item[name]['is_depth'])
                ser = ImageORBSerializer(topic_name, is_depth, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))

            elif type == "Odometry":
                ser = OdomSerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))

            elif type == "Pose":
                ser = PoseSerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))

            elif type == "CamInfo":
                ser = CamInfoSerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))

            elif type == "FloatArray":
                ser = FloatArrSerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))

            elif type == "IntArray":
                ser = IntArrSerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))

            elif type == "Twist":
                ser = TwistSerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))
            elif type == "Map":
                ser = MapSerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))
            elif type == "Path":
                ser = PathSerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))
            elif type == "ToeArray":
                ser = ToeArraySerializer(topic_name, directory_name=output_dir_name)
                rospy.logdebug("Created {} Serializer listening to {}".format(type, topic_name))

            else:
                rospy.logerr_once("[BHARVEST] Type {} is not an implemented "
                                  "serializer type.".format(type))
                continue

            serializer_list.append(ser)

    return serializer_list


if __name__ == '__main__':
    # arg parse lots of stuff
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("--live", help="Run live without rosbag", default=False)
    parser.add_argument("--bag_file", help="Input ROS bag.")
    parser.add_argument("--bag_file_list", help="Text file with list of rosbags", default=None)
    parser.add_argument("--output_dir", help="Output directory.", default="./data")
    parser.add_argument("--config_file", help="Config File", default="harvester_configs/simple_learned_dynamics.yaml")

    args = parser.parse_args()

    bags = []
    if args.bag_file_list is not None:
        bags = open(args.bag_file_list, 'r').readlines()
    else:
        bags = [args.bag_file]


    if not args.live:

        for bag in bags:
            # start roscore and init the rosnode
            roscore_process = subprocess.Popen("roscore")

            time.sleep(1)
            rospy.init_node("bag_harvester", anonymous=True)
            bag_name = bag.split("/")[-1]
            # start the rosbag to serialize from, but only if we havent done it yet
            if bag_name.split("/")[-1][:-5] in list(os.listdir(args.output_dir)):
                print("We did this one already:", bag_name)
                subprocess.call(['pkill', '-f', 'ros'])
                continue

            # ----------- serializing -------------------
            # this creates a bunch of subscribers, needs to be after rospy.init
            serializer_list = process_yaml(args.config_file, args.output_dir, bag_name)

            rosbag_process = subprocess.run(["rosbag", "play", "--clock", "-r", "0.5", bag.strip()])
            # rosbag_process = subprocess.run(["rosbag", "info", bag.strip()])

            # we did it!
            rospy.logwarn("Completed return code for rosbag: {}".format(rosbag_process.returncode))

            # now kill everything
            if rosbag_process.returncode == 0:
                rospy.logwarn("Completed parsing {} successfully!".format(bag_name))

            subprocess.Popen(['pkill', '-f', 'ros'])
            print("Killed all ROS")

    else:
        time.sleep(1)
        rospy.init_node("bag_harvester", anonymous=True)

        serializer_list = process_yaml(args.config_file, args.output_dir, "live")

        while not rospy.is_shutdown():
            time.sleep(1)
