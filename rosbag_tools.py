"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import yaml
import rosbag
from rosbag.bag import Bag
import glob
import argparse
import math


def truncate(number, digits) -> float:
    stepper = 10.0 ** digits
    return math.trunc(stepper * number) / stepper


def pretty_list(l, indent=0):
    for value in l:
        if isinstance(value, dict):
            pretty_dict(value, indent + 1)
        elif isinstance(value, list):
            pretty_list(value, indent + 1)
        else:
            print('\t' * (indent + 1) + str(value))


def pretty_dict(d, indent=0):
    for key, value in d.items():
        if key == "types":
            continue
        print('\t' * indent + str(key))
        if isinstance(value, dict):
            pretty_dict(value, indent + 1)
        elif isinstance(value, list):
            pretty_list(value, indent + 1)
        else:
            print('\t' * (indent + 1) + str(value))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("directory", help="Directory with rosbags")
    parser.add_argument("--print", default=False, help="print out the yamls")
    parser.add_argument("--topic", default=None, help="If you want bags with a specific topic, provide it here")
    parser.add_argument("--write", default=False)
    parser.add_argument("--write_file", default="bags.txt")

    args = parser.parse_args()

    topic_count = {}
    topic_total_msgs = {}
    bag_files = glob.glob(args.directory + "*.bag")
    for bag in bag_files:
        info_dict = yaml.load(Bag(bag, 'r')._get_yaml_info())

        if args.print:
            pretty_dict(info_dict)
            print("-" * 40)

        for topic_dict in info_dict["topics"]:
            topic = topic_dict["topic"]
            if topic not in topic_count:
                topic_count[topic] = []
                topic_total_msgs[topic] = 0
            topic_count[topic].append(bag)
            topic_total_msgs[topic] += topic_dict["messages"]

    f = None
    if args.write:
        f = open("summary_" + args.write_file, 'w')

    print("Overall Topic Counts: ")
    f.write("Overall Topic Counts:\n")

    print("-" * 50)
    f.write("-" * 50 + "\n\n")

    for key, val in topic_count.items():
        print("|", key, (40 - len(key)) * " ", "|", len(val), "\t|", truncate(len(val) / len(bag_files) * 1.0, 2),
              "\t|", topic_total_msgs[key])

        f.write("|" + key + (40 - len(key)) * " " + "|" + str(len(val)) + "\t|" + str(truncate(len(val) / len(bag_files) * 1.0, 2)) + "\t|" +
                str(topic_total_msgs[key]) + "\n")

    print("-" * 50, "\n\n")
    f.write("-" * 50 + "\n\n")

    if args.topic is not None:
        print("Bags with {} in them".format(args.topic))

        f = None
        if args.write:
            f = open(args.topic[1:].replace("/", "_") + "_" + args.write_file, 'w')
        for bag in topic_count[args.topic]:
            print(bag)
            if args.write:
                f.write(bag + "\n")
    print("-" * 50, "\n\n")
