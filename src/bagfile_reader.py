import json
import os
import csv

import bagpy
from matplotlib import pyplot as plt
import pandas as pd
from bagpy import bagreader
import numpy as np


def bagfile_to_csvs(bagfile):
    """
    Reads a bagfile and extracts topics into dictionaries
    """

    bag = bagreader(bagfile)

    # --- Read the times when events where triggered ---
    data = bag.message_by_topic("experiment_steps")
    data_list = pd.read_csv(data)

    # Extract x-axis, usually time
    events_time_stamp = data_list.iloc[:, 0]
    events_time = elapsed_time(events_time_stamp)

    event_values = data_list.iloc[:, 1]

    print(event_values)
    print(events_time)
    print(events_time_stamp)

    # --- Get the topics available in the bagfile
    " print the list of topics"
    topics = bag.topic_table
    print("Bagfile topics:\n", topics)

    # --- Sweep all topics, plot and add the events
    for topic, topic_type in zip(topics['Topics'], topics['Types']):

        # Skip images
        if topic_type == "sensor_msgs/Image" or topic == "experiment_steps":
            pass
        else:
            print("\n", topic)
            data = bag.message_by_topic(topic)
            data_list = pd.read_csv(data)

            # Check how many values per topic
            print(data_list.shape)

            if topic == "wrench":
                time, force = bag_plot_wrench(data_list)
                plt.figure()
                plt.plot(time, force)

                # Add vertical lines at the events
                for event, label in zip(events_time, event_values):
                    plt.axvline(x=event, color='red', linestyle='dotted', linewidth=2)
                    plt.text(event, 0, label, rotation=90)

                plt.ylim([-22, 10])

                plt.xlabel('elapsed time [s]')
                plt.ylabel(topic)
                plt.title(bagfile)
                plt.grid()

            elif topic == "joint_states":
                pass
            else:
                # Extract x-axis, usually time
                time_stamp = data_list.iloc[:, 0]
                elapsed = elapsed_time(time_stamp)
                values = data_list.iloc[:, 1]

                plt.figure()
                # plt.plot(elapsed, values)
                plt.plot(time_stamp, values)

                # Add vertical lines at the events
                for event, label in zip(events_time_stamp, event_values):
                    plt.axvline(x=event, color='red', linestyle='dotted', linewidth=2)
                    plt.text(event, 600, label, rotation=90)

                plt.ylim([0, 1200])
                plt.xlabel('elapsed time [s]')
                plt.ylabel(topic)
                plt.title(bagfile)
                plt.grid()

        dictios = 0

    return dictios


def bag_to_csvs():
    """Method to open all the bagfiles in a folder and save all topics as csvs"""

    location = os.path.dirname(os.getcwd())
    foldername = "/data/data_3/"

    for file in os.listdir(location + foldername):
        if file.endswith(".bag"):
            bag = bagreader(location + foldername + file)
            print("\n\n", file)

            # --- Get the topics available in the bagfile
            topics = bag.topic_table
            print("Bagfile topics:\n", topics)

            # --- Read the desired topic
            for topic in topics["Topics"]:
                data = bag.message_by_topic(topic)

                # Once open, the data is saved automatically into a csv file


def read_jsons(foldername=""):

    location = os.path.dirname(os.getcwd())
    experiments = []
    id = 0
    for file in os.listdir(location + foldername):
        if file.endswith(".json"):
            id += 1
            json_file = open(location + foldername + file)
            json_data = json.load(json_file)
            experiment = Experiment(id)
            experiment.exp_type = json_data["generalInfo"]["experimentType"]
            try:
                experiment.pressure = json_data["gripperInfo"]["pressureAtValve [PSI]"]
            except KeyError:
                experiment.pressure = json_data["gripperInfo"]["pressureAtValve"]

            experiment.surface = json_data["surfaceInfo"]["type"]
            experiment.file_source = file

            # print(experiment.exp_type)
            experiments.append(experiment)

    return experiments


def read_csvs(experiments, foldername=""):

    for experiment in experiments:

        filename = experiment.file_source
        subfolder = filename.split('.json')[0]
        # print(subfolder)
        location = os.path.dirname(os.getcwd())
        new_location = location + foldername + subfolder + "/"

        # Sweep the csvs of each experiment
        for file in os.listdir(new_location):
            data_list = pd.read_csv(new_location + file)

            if file == "gripper-pressure.csv":
                experiment.pressure_time_stamp = data_list.iloc[:, 0]
                experiment.pressure_values = data_list.iloc[:, 1]

            if file == "rench.csv":
                experiment.wrench_time_stamp = data_list.iloc[:, 0]
                experiment.wrench_zforce_values = data_list.iloc[:, 7]

            if file == "xperiment_steps.csv":
                experiment.event_time_stamp = data_list.iloc[:, 0]
                experiment.event_values = data_list.iloc[:, 1]

        experiment.elapsed_times()

    return experiments


class Experiment():
    """This class is to define experiments as objects.
    Each experiment has properties from the json file.

    """
    def __init__(self, id=0,
                 exp_type="vertical",
                 pressure=60,
                 surface="3DPrinted_with_Primer",
                 radius=37.5,
                 z_noise=0,
                 x_noise=0,
                 file_source=""):
        self.exp_type = exp_type
        self.pressure = pressure
        self.surface = surface
        self.surface_radius = radius
        self.z_noise = z_noise
        self.x_noise = x_noise
        self.time_stamp = []
        self.pressure_values = []
        self.id = id
        self.file_source = file_source

        # Lists to save the data from csvs
        self.pressure_time_stamp = []
        self.pressure_elapsed_time = []
        self.pressure_values = []
        self.wrench_time_stamp = []
        self.wrench_elapsed_time = []
        self.wrench_zforce_values = []
        self.event_time_stamp = []
        self.event_elapsed_time = []
        self.event_values = []
        self.first_time_stamp = 0

    def initial_stamp(self):
        try:
            self.first_time_stamp = min(min(self.pressure_time_stamp), min(self.wrench_time_stamp), min(self.event_time_stamp))
        except ValueError:
            self.first_time_stamp = 0

    def elapsed_times(self):

        # First Obtain the initial time stamp of the experiment as a reference to the rest
        self.initial_stamp()

        self.pressure_elapsed_time = [None] * len(self.pressure_time_stamp)
        for i in range(len(self.pressure_time_stamp)):
            self.pressure_elapsed_time[i] = self.pressure_time_stamp[i] - self.first_time_stamp

        self.wrench_elapsed_time = [None] * len(self.wrench_time_stamp)
        for i in range(len(self.wrench_time_stamp)):
            self.wrench_elapsed_time[i] = self.wrench_time_stamp[i] - self.first_time_stamp

        self.event_elapsed_time = [None] * len(self.event_time_stamp)
        for i in range(len(self.event_time_stamp)):
            self.event_elapsed_time[i] = self.event_time_stamp[i] - self.first_time_stamp

    def vacuum_reached(self):
        ...


def main():

    # --- Read bagfile
    foldername = "/data/data_1/"

    # --- Step1: Turn Bag into csvs if needed
    # bag_to_csvs(foldername)

    # --- Step2: Read attributes from json files
    metadata = read_jsons(foldername)

    # --- Step3: Read values from the csv files for each json file
    experiments = read_csvs(metadata, foldername)

    # --- Step4: Get different properties from each experiment
    #mean value of vacuum during steady state


    # --- Step4: Plot
    x = experiments[0].wrench_elapsed_time
    y = experiments[0].wrench_zforce_values
    x = experiments[0].pressure_elapsed_time
    y = experiments[0].pressure_values

    event_x = experiments[0].event_elapsed_time
    event_y = experiments[0].event_values
    plt.figure()
    plt.plot(x,y)

    # Add vertical lines at the events
    for event, label in zip(event_x, event_y):
        plt.axvline(x=event, color='red', linestyle='dotted', linewidth=2)
        plt.text(event, 600, label, rotation=90)

    # plt.ylim([0, 1200])
    plt.xlabel('elapsed time [s]')
    plt.title(experiments[0].file_source)
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()

