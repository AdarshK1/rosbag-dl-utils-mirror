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

        self.actual_end_time = int(min(np.array(self.end_times)))
        self.actual_time_length = self.actual_end_time - self.skip_last_n_seconds - self.skip_first_n_seconds
        self.num_samples = self.actual_time_length * self.samples_per_second

        self.frequencies = {}
        for key in self.data_holder.keys():
            self.frequencies[key] = len(self.data_holder[key]) / sorted(list(self.data_holder[key].keys()))[-1]

        # print(self.frequencies)

    def setup_data_holders(self, verbose=True):
        for topic_dir in self.topic_dirs:
            self.data_holder[topic_dir] = {}
            t1 = time.time()
            filenames = sorted(glob.glob(self.data_dir + topic_dir + "/*"))
            t2 = time.time()
            print("Parsing Topic: {}, Count: {}".format(topic_dir, len(filenames)))
            print("Sort time:", t2 - t1)

            nbytes = 0
            offset = 0
            for idx, item in enumerate(filenames):
                if idx == 0:
                    nbytes = os.stat(item).st_size
                    offset = int(nbytes) - 8
                tf1 = time.time()
                mm = np.memmap(item, np.float, 'r', offset)
                # print(int(a.st_size))
                # datum = np.load(item, allow_pickle=True, mmap_mode='r')
                # print(datum[-1], mm[0])
                # tf2 = time.time()
                timing = mm[0]
                # timing = datum[-1]
                if idx == 0:
                    self.initial_times[topic_dir] = timing
                tf3 = time.time()
                self.data_holder[topic_dir][timing - self.initial_times[topic_dir]] = item
                tf4 = time.time()
                self.raw_end_times[topic_dir] = timing
                # del datum

                if verbose:
                    if idx % (len(filenames) // 10) == 0:
                        pcent_complete = round(idx / len(filenames) * 100)
                        print(pcent_complete, "%")
                tflast = time.time()

                # print("load: {:.8f}".format(tf2 - tf1))
                # print("if: {:.8f}".format(tf3 - tf2))
                # print("set item: {:.8f}".format(tf4 - tf3))
                # print("end time verbose: {:.8f}".format(tflast - tf4))

            t3 = time.time()
            self.data_list_holder[topic_dir] = list(self.data_holder[topic_dir].items())
            t_last = time.time()
            print("dict to list: {:.48}".format(t_last - t3))

            print("Full topic: {:.8f}".format(t_last - t1))
            print("\n")

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
                # t1 = time.time()
                # indices[topic_dir] = (max([i for i, (k, v) in enumerate(self.data_list_holder[topic_dir]) if
                #                            k <= (idx * self.actual_time_length / self.num_samples)]))
                # t2 = time.time()
                # print("list comp time\t {:.8f}".format(t2-t1))
                #
                # max_i = 0
                time_per_sample = self.actual_time_length / self.num_samples
                time_point = idx * time_per_sample
                # idx = k / time_per_sample
                # for i, (k, v) in enumerate(self.data_list_holder[topic_dir]):
                #     if k > time_point:
                #         max_i = i - 1
                #         break
                t3 = time.time()
                # print("for loop time\t {:.8f}".format(t3-t2))

                approx_i = int(time_point * self.frequencies[topic_dir])

                N_seconds = 5

                max_check = np.clip(approx_i + int(self.frequencies[topic_dir]) * N_seconds, 0, len(self.data_list_holder[topic_dir]) - 1)
                min_check = np.clip(approx_i - int(self.frequencies[topic_dir]) * N_seconds, 0, len(self.data_list_holder[topic_dir]) - 1)
                max_i_v2 = 0
                for j in range(min_check, max_check):
                    k = self.data_list_holder[topic_dir][j][0]
                    if k > time_point:
                        max_i_v2 = j - 1
                        break
                indices[topic_dir] = max_i_v2

                t4 = time.time()
                # print("approx time\t {:.8f}".format(t4-t3))

                # print("results", indices[topic_dir], max_i, max_i_v2)

            except ValueError:
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
