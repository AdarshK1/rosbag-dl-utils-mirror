# ROSbag DL Utilities

This repo is a collection of python utilities that help expedite the process of turning ROS bags into numpy and torch
friendly files while maintaining chronological information (something quite often critical to any robotics applications).

## Bag Harvester

Using yaml files, a couple examples of which can be found in the `harvester_configs` directory, users can create a logging
config that specifies what topics of what type need to be logged and saved out. The `bag_harvester.py` script then takes
that config file in, along with an output directory and bag file (or list of bag files) and processes them to create
directories of numpy arrays that are easy to ingest in a data loader and pass through to a training pipeline.

The serializers, found in the `serializers` directory are all short classes, all extending the `base_serializer` class,
that wrap the basic conversions needed to subscribe to a topic and go from some kind of ros msg, like Image or Map or PoseStamped, to a numpy array.

The bag harvester script instantiates these serializers based on the yaml file, and works through the entire rosbag, while also maintaing the ROS time stamp on all of the messages.


A couple of notes on the bag harvester:
* The script can handle a single bag file, a list of bag files, or it can even run live (skipping rosbagging alltogether)
* The script does not use the rosbag python API, but instead actually uses subprocess to start a roscore and play the bag
  * This was done so that if a user wants to run some other software offline (say a mapper) while playing the rosbag of just sensor data, they can then also log the output of the offline software without originally bagging it.

## Base Data Loader

Now that the bag harvester has created a directory of numpy arrays that store all of the messages from the rosbag, we need
some unified data loader scheme. That's where `base_data_loader.py` comes in, providing a base class that can be extended
which has the built in functionality of organizing all messages by time, allowing sampling through time of all the data in the rosbag and many other helpful features.

A short summary of the behavior of the base data loader:
* Organize all messages from the harvested rosbag chronologically
* Given multiple topics of varying publish frequency, use a single unified sampling frequency and return the most recent message given that sample location
* Blacklist sections of time (currently only at the start and end, but specific blocks coming)

