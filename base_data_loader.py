"""
Copyright (C) Ghost Robotics - All Rights Reserved
Written by Adarsh Kulkarni <adarsh@ghostrobotics.io>
"""

import numpy as np
import torch
import glob
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, utils
import copy
import cv2
import yaml
from serializers.Base_Serializer import BaseSerializer
import time
import os


class BaseDataset(Dataset):

    def __init__(self, config_file, sub_dir, samples_per_second=30, skip_last_n_seconds=5, skip_first_n_seconds=2):

        self.data_dir = sub_dir

        self.topic_dirs = self.read_config_file(config_file)
        print(self.topic_dirs)

        self.data_holder = {}
        self.data_list_holder = {}
        self.initial_times = {}
        self.raw_end_times = {}

        self.setup_data_holders()

        self.end_times = [sorted(list(self.data_holder[key].keys()))[-1] for key in self.data_holder.keys()]

        self.samples_per_second = samples_per_second

        # this is super inconvenient, but it works
        self.skip_last_n_seconds = skip_last_n_seconds
        self.skip_first_n_seconds = skip_first_n_seconds

        self.actual_end_time = int(min(np.array([i for i in self.end_times if i > 0.1])))

        # print(self.end_times)
        self.actual_time_length = self.actual_end_time - self.skip_last_n_seconds - self.skip_first_n_seconds
        self.num_samples = self.actual_time_length * self.samples_per_second

        self.frequencies = {}
        for key in self.data_holder.keys():
            self.frequencies[key] = len(self.data_holder[key]) / (sorted(list(self.data_holder[key].keys()))[-1] + 1E-3)

        # print("init", self.frequencies)

    def setup_data_holders(self, verbose=True):
        for topic_dir in self.topic_dirs:
            self.data_holder[topic_dir] = {}
            filenames = sorted(glob.glob(self.data_dir + topic_dir + "/*"))
            print("Parsing Topic: {}, Count: {}".format(topic_dir, len(filenames)))
            for idx, item in enumerate(filenames):
                if "map" in item or "odom" in item:
                    nbytes = os.stat(item).st_size
                    offset = int(nbytes) - 8
                    mm = np.memmap(item, np.float, 'r', offset)

                    timing = mm[0]
                else:
                    loaded = np.load(item, allow_pickle=True)
                    timing = loaded[-1]
                    del loaded

                if idx == 0:
                    self.initial_times[topic_dir] = timing

                self.data_holder[topic_dir][timing - self.initial_times[topic_dir]] = item
                self.raw_end_times[topic_dir] = timing

                if verbose and len(filenames) > 10:
                    if idx % (len(filenames) // 10) == 0 and idx > 0:
                        pcent_complete = round(idx / len(filenames) * 100)
                        print(pcent_complete, "%")

            self.data_list_holder[topic_dir] = list(self.data_holder[topic_dir].items())

    def read_config_file(self, config_file):
        names = []
        with open(config_file, 'r') as stream:
            try:
                parsed_yaml = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc.__str__())

            for item in parsed_yaml:
                name = list(item.keys())[0]
                topic_name = item[name]["topic_name"]
                bs = BaseSerializer(topic_name)
                name = bs.filename_base.split("/")[2]
                names.append(name)
        return names

    def __len__(self):
        return self.num_samples

    def __getitem__(self, idx):
        # print("idx", idx)
        idx = idx + self.skip_first_n_seconds * self.samples_per_second
        # print("idx", idx)

        # TODO (akulkarni) comment this bs
        indices = {}
        vals = {}
        for topic_dir in self.topic_dirs:
            try:
                time_per_sample = self.actual_time_length / self.num_samples
                time_point = idx * time_per_sample
                approx_i = int(time_point * self.frequencies[topic_dir])

                N_seconds = 5

                max_check = np.clip(approx_i + int(self.frequencies[topic_dir]) * N_seconds, 0,
                                    len(self.data_list_holder[topic_dir]) - 1)
                min_check = np.clip(approx_i - int(self.frequencies[topic_dir]) * N_seconds, 0,
                                    len(self.data_list_holder[topic_dir]) - 1)
                max_i_v2 = 0
                # print(min_check, max_check)
                for j in range(min_check, max_check + 1):
                    k = self.data_list_holder[topic_dir][j][0]
                    if k > time_point:
                        max_i_v2 = j - 1
                        break
                indices[topic_dir] = max_i_v2

            except ValueError:
                print(self.frequencies)
                print("idx", idx, "time_per_sample", self.actual_time_length / self.num_samples, "self",
                      self.frequencies[topic_dir], topic_dir)

                print("Error: ",
                      topic_dir,
                      idx,
                      self.samples_per_second,
                      self.num_samples,
                      self.actual_time_length,
                      (idx * self.actual_time_length / self.num_samples))

            filename = self.data_list_holder[topic_dir][indices[topic_dir]][1]
            # print(filename)
            # filename = self.data_holder[topic_dir].items()[indices[topic_dir]][1]
            datum = np.load(filename, allow_pickle=True)
            vals[topic_dir] = datum[:-1]

        return vals


if __name__ == "__main__":
    # np.random.seed(10)
    N = 10
    sample_fname = "/home/adarsh/SSD1/HarvestedData/v4_dynamics/"
    sample_fname = "/home/adarsh/SSD1/HarvestedData/v4_dynamics_overnight/live/"
    config_file = "harvester_configs/simple_learned_dynamics.yaml"
    dset = BaseDataset(config_file, sample_fname)
    for i in range(N):
        vals = dset.__getitem__(i)
        # print(vals)
    vals = dset.__getitem__(dset.num_samples - 1)
