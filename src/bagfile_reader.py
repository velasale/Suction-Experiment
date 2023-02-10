import os
import csv

import bagpy
from matplotlib import pyplot as plt
import pandas as pd
from bagpy import bagreader
import numpy as np


def elapsed_time(time_stamp, offset=0):
    """
    Simplifies the time axis, by subtracting the initial time.
    This is useful because usually the time stamps are given in a long format (i.e. in the order of 1e9)
    :param variable: Reference variable to obtain the size of the time array
    :param time_stamp: The time stamp array that is going to be simplified
    :return: Simplified time as Elapsed Time
    """
    elapsed = [None] * len(time_stamp)
    for i in range(len(time_stamp)):
        # elapsed[i] = time_stamp[i] - 1 * time_stamp[0] + offset
        elapsed[i] = time_stamp[i]
    return elapsed


def bag_plot_wrench(data):

    wrench_time_stamp = data.iloc[:, 0]
    forces_x = data.iloc[:, 5]
    forces_y = data.iloc[:, 6]
    forces_z = data.iloc[:, 7]
    torques_x = data.iloc[:, 8]
    torques_y = data.iloc[:, 9]
    torques_z = data.iloc[:, 10]

    wrench_elapsed_time = elapsed_time(wrench_time_stamp)

    return wrench_elapsed_time, forces_z


class Experiment():
    """This class is to define experiments as objects.
    Each experiment has properties from the json file.

    """
    def __init__(self, exp_type="vertical",
                 pressure=60,
                 surface="3DPrinted_with_Primer",
                 radius=37.5):
        self.exp_type = exp_type
        self.pressure = pressure
        self.surface = surface
        self.surface_radius = radius


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


def main():

    # --- Read bagfile
    # location = os.path.dirname(os.getcwd())
    # foldername = "/data/simple_suction/"
    # filename = "simple_suction_#0_pres_50_surface_Gloss_Fake_Apple_radius_0.0375.bag"
    #
    # # --- Step1: Convert bagfiles into pkl
    # parara = bagfile_to_csvs(location+foldername+filename)
    # plt.show()

    # --- Step1: Turn Bag into csvs if needed
    bag_to_csvs()

    # --- Step2:

if __name__ == '__main__':
    main()

