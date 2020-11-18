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
from bag_harvester.Base_Serializer import BaseSerializer

# TODO(akulkarni) i realized this whole setup requires that the dataset is held in RAM...that kinda sucks
# with big datasets so fix that sometime soon (should be doable just refactoring tbh)


class BaseDataset(Dataset):

    def __init__(self, config_file, sub_dir, samples_per_second=30, skip_last_n_seconds=5, skip_first_n_samples=15):

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
        self.skip_first_n_samples = skip_first_n_samples

        self.actual_end_time = int(min(np.array(self.end_times)))
        self.actual_time_length = self.actual_end_time - self.skip_last_n_seconds
        self.num_samples = self.actual_time_length * self.samples_per_second - self.skip_first_n_samples

    def setup_data_holders(self):
        for topic_dir in self.topic_dirs:
            self.data_holder[topic_dir] = {}
            print("Parsing Topic: {}".format(topic_dir))
            for idx, item in enumerate(sorted(glob.glob(self.data_dir + topic_dir + "/*"))):

                datum = np.load(item, allow_pickle=True)
                if idx == 0:
                    self.initial_times[topic_dir] = datum[-1]

                self.data_holder[topic_dir][datum[-1] - self.initial_times[topic_dir]] = datum[:-1]

                self.raw_end_times[topic_dir] = datum[-1]

            self.data_list_holder[topic_dir] = list([(k, v) for k, v in self.data_holder[topic_dir].items()])

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

        idx = idx + self.skip_first_n_samples

        # TODO (akulkarni) comment this bs
        indices = {}
        vals = {}
        for topic_dir in self.topic_dirs:
            try:
                indices[topic_dir] = (max([idx for idx, (k, v) in enumerate(self.data_holder[topic_dir].items()) if
                                           k <= (idx * self.actual_time_length / self.num_samples)]))
            except ValueError:
                print("Error: ",
                      topic_dir,
                      idx,
                      self.samples_per_second,
                      self.num_samples,
                      self.actual_time_length,
                      (idx * self.actual_time_length / self.num_samples))
            vals[topic_dir] = (self.data_list_holder[topic_dir][indices[topic_dir]][1])

        return vals


if __name__ == "__main__":
    # np.random.seed(10)
    N = 1
    sample_fname = "/home/adarsh/SSD1/HarvestedData/v4_dynamics/"
    config_file = "harvester_configs/simple_learned_dynamics.yaml"
    dset = BaseDataset(config_file, sample_fname)
    for i in range(N):
        vals = dset.__getitem__(0)
        # for k, v in vals.items():
        #     print(k, v.shape)
