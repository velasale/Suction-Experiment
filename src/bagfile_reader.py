import os
import csv

import bagpy
from matplotlib import pyplot as plt
import pandas as pd
from bagpy import bagreader
import numpy as np



def read_events():
    ...
def bagfile_to_csvs(bagfile):
    """
    Reads a bagfile and extracts topics into dictionaries
    """

    bag = bagreader(bagfile)

    # --- Read the times when events where triggered ---
    data = bag.message_by_topic("experiment_steps")
    data_list = pd.read_csv(data)

    # Extract x-axis, usually time
    time_stamp = data_list.iloc[:, 0]
    events_elapsed_time = [None] * len(time_stamp)
    for i in range(len(time_stamp)):
        events_elapsed_time[i] = time_stamp[i] - time_stamp[0]
    event_values = data_list.iloc[:, 1]


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
                pass
            elif topic == "joint_states":
                pass
            else:
                # Extract x-axis, usually time
                time_stamp = data_list.iloc[:, 0]
                elapsed_time = [None] * len(time_stamp)
                for i in range(len(time_stamp)):
                    elapsed_time[i] = time_stamp[i] - time_stamp[0]

                values = data_list.iloc[:, 1]

                plt.figure()
                plt.plot(elapsed_time, values)

                plt.xlabel('elapsed time [s]')
                plt.ylabel(topic)
                plt.title(bagfile)
                plt.grid()

        dictios = 0

    return dictios


def main():

    # --- Read bagfile
    location = os.path.dirname(os.getcwd())        
    foldername = "/data_x_noise/"
    filename = "horizontal_#7_pres_80_surface_3DprintedPLA_radius_0.0375_noise_0.44.bag"

    parara = bagfile_to_csvs(location+foldername+filename)


    plt.show()


if __name__ == '__main__':
    main()

