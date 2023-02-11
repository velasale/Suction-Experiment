import json
import os
import csv

import bagpy
from matplotlib import pyplot as plt
import pandas as pd
from bagpy import bagreader
import numpy as np


def bag_to_csvs(foldername):
    """Method to open all the bagfiles in a folder and save all topics as csvs"""

    location = os.path.dirname(os.getcwd())

    for file in os.listdir(location + foldername):
        if file.endswith(".bag"):
            bag = bagreader(location + foldername + file)
            # print("\n\n", file)

            # --- Get the topics available in the bagfile
            topics = bag.topic_table
            # print("Bagfile topics:\n", topics)

            # --- Read the desired topic
            for topic in topics["Topics"]:
                data = bag.message_by_topic(topic)

                # Once open, the data is saved automatically into a csv file


def read_jsons(foldername=""):

    location = os.path.dirname(os.getcwd())
    experiments = []
    id = 0
    for file in sorted(os.listdir(location + foldername)):
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
            try:
                experiment.surface_radius = json_data["surfaceInfo"]["radius [m]"]
            except KeyError:
                experiment.surface_radius = json_data["surfaceInfo"]["radius"]
            experiment.file_source = file
            experiment.x_noise = abs(json_data["robotInfo"]["x noise real [m]"])
            experiment.z_noise = abs(json_data["robotInfo"]["z noise real [m]"])

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
        for file in sorted(os.listdir(new_location)):
            data_list = pd.read_csv(new_location + file)

            if file == "gripper-pressure.csv":
                experiment.pressure_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.pressure_values = data_list.iloc[:, 1].tolist()

            if file == "rench.csv":
                experiment.wrench_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.wrench_zforce_values = data_list.iloc[:, 7].tolist()

            if file == "xperiment_steps.csv":
                experiment.event_time_stamp = data_list.iloc[:, 0].tolist()
                experiment.event_values = data_list.iloc[:, 1].tolist()

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

        # Experiment initial attributes
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
        self.atmospheric_pressure = 0
        self.testrig_weight = 0

        # Statistical Features
        self.steady_pressure_values = []
        self.steady_vacuum_mean = 0
        self.steady_vacuum_std = 0
        self.max_detach_zforce = 0

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

    def get_steady_vacuum(self):
        """Method to obtain the mean and std deviation of the vacuum during steady state"""

        # Get the index at which the steady state starts and ends
        start_index = self.event_values.index('Steady')
        end_index = self.event_values.index("Retrieve")

        # Get the time at which the steady state starts and ends
        steady_vacuum_start = self.event_elapsed_time[start_index]
        steady_vacuum_end = self.event_elapsed_time[end_index]
        # print("\n %.0d Starts at %.2f and ends at %.2f" %(self.id, steady_vacuum_start, steady_vacuum_end))

        # Get the steady state mean and std values
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if (time > steady_vacuum_start) and (time < steady_vacuum_end):
                self.steady_pressure_values.append(value)

        self.steady_vacuum_mean = np.mean(self.steady_pressure_values)
        self.steady_vacuum_std = np.std(self.steady_pressure_values)

        # print(self.steady_vacuum_mean, self.steady_vacuum_std)

    def get_detach_force(self):
        """Method to obtain the mean and std deviation of the vacuum during the retrieval
        """
        # Get the index at which the steady state starts and ends
        start_index = self.event_values.index('Retrieve')
        end_index = self.event_values.index("Vacuum Off")

        # Get the time at which the steady state starts and ends
        retrieve_start = self.event_elapsed_time[start_index]
        vacuum_stops = self.event_elapsed_time[end_index]

        # Get the detachment values
        detach_values = []
        for time, value in zip(self.wrench_elapsed_time, self.wrench_zforce_values):
            if (time > retrieve_start) and (time < vacuum_stops):
                detach_values.append(value)

        self.max_detach_zforce = max(detach_values)

        # print("\n %.0d Maximum detachment force %.2f" % (self.id, self.max_detach_zforce))


def main():

    plt.figure()
    pressures = [50, 60, 70, 80]

    # Conditions for the plot
    # radius = 0.0425
    radius = 0.0375
    # pressure = 80

    for pressure in pressures:

        list_of_means = []
        list_of_stds = []
        list_of_x_noises = []
        list_of_z_noises = []
        for i in range(3):

            foldername = "/data/z_noise/rep" + str(i+1) + "/"

            # --- Step1: Turn Bag into csvs if needed
            # bag_to_csvs(foldername)

            # --- Step2: Read attributes from json files
            metadata = read_jsons(foldername)

            # --- Step3: Read values from the csv files for each json file
            experiments = read_csvs(metadata, foldername)

            # --- Step4: Get different properties from each experiment
            # TODO Fix why is the std so small
            # TODO Fix avoid values that didn't touch the surface
            # TODO mind about the initial atmospheric pressure
            # TODO mind about the initial weight measurement

            x_noises = []
            z_noises = []
            vacuum_means = []
            vacuum_stds = []

            for experiment in experiments:
                # Only take those experiments that meet the conditions
                if experiment.surface_radius == radius and experiment.pressure == pressure:

                    experiment.get_steady_vacuum()
                    experiment.get_detach_force()

                    x_noise = experiment.x_noise
                    z_noise = experiment.z_noise

                    steady_vacuum_mean = experiment.steady_vacuum_mean
                    steady_vacuum_std = experiment.steady_vacuum_std

                    # print(experiment.file_source)
                    # print(steady_vacuum_mean, steady_vacuum_std)
                    # print(x_noise, z_noise)

                    x_noises.append(x_noise)
                    z_noises.append(z_noise)
                    vacuum_means.append(steady_vacuum_mean)
                    vacuum_stds.append(steady_vacuum_std)

            list_of_means.append(vacuum_means)
            list_of_stds.append(vacuum_stds)
            list_of_x_noises.append(x_noises)
            list_of_z_noises.append(z_noises)

        print("\nMeans: %.2f", list_of_means)
        print("Stds: %.2f", list_of_stds)
        print("x-noises: ", list_of_x_noises)

        # Elaborate the final means
        final_means = []
        final_stds = []
        final_x_noises = []
        final_z_noises = []
        for j in range(10):
            mean_vals = []
            std_vals = []
            x_noises_vals = []
            z_noises_vals = []
            for i in range(3):
                mean_vals.append(list_of_means[i][j])
                std_vals.append(list_of_stds[i][j])
                x_noises_vals.append(list_of_x_noises[i][j])
                z_noises_vals.append(list_of_z_noises[i][j])
            # print(vals)
            final_means.append(np.mean(mean_vals))
            final_x_noises.append(np.mean(x_noises_vals))
            final_z_noises.append(np.mean(z_noises_vals))
            # Mean of standards
            a = std_vals[0]
            b = std_vals[1]
            c = std_vals[2]
            mean_stds = ((a ** 2 + b ** 2 + c ** 2) / 3) ** 0.5
            final_stds.append(mean_stds)

        print(final_means)
        print(final_stds)
        print(final_x_noises)
        print(final_z_noises)

        # plt.errorbar(final_x_noises, final_means, final_stds, label=(str(pressure)+" PSI"))
        plt.errorbar(final_z_noises, final_means, final_stds, label=(str(pressure) + " PSI"))

    title = "Cartesian noise in z, for %.4f radius" % (radius)
    plt.xlabel("z-noise [m]")
    plt.ylabel("Vacuum [hPa]")
    plt.ylim([100, 1100])
    plt.xlim([0, 0.030])
    plt.legend()
    plt.grid()
    plt.title(title)
    plt.show()

        #     # --- Step4: Plot
        #     x = experiments[id].wrench_elapsed_time
        #     y = experiments[id].wrench_zforce_values
        #     # x = experiments[id].pressure_elapsed_time
        #     # y = experiments[id].pressure_values
        #
        #     event_x = experiments[id].event_elapsed_time
        #     event_y = experiments[id].event_values
        #     plt.figure()
        #     plt.plot(x, y)
        #
        #     # Add vertical lines at the events
        #     for event, label in zip(event_x, event_y):
        #         plt.axvline(x=event, color='red', linestyle='dotted', linewidth=2)
        #         plt.text(event, np.mean(y), label, rotation=90)
        #
        #     # plt.ylim([0, 1200])
        #     plt.xlabel('elapsed time [s]')
        #     plt.title(experiments[0].file_source)
        #     plt.grid()
        #
        # plt.show()


if __name__ == '__main__':
    main()

