import json
import os
import re
import csv

import bagpy
from matplotlib import pyplot as plt
import pandas as pd
from bagpy import bagreader
import numpy as np


def bag_to_csvs(file):
    """Open all bagfiles in a folder and saves all topics as csvs"""

    if file.endswith(".bag"):
        bag = bagreader(file)
        # print("\n\n", file)

        # --- Get the topics available in the bagfile ---
        topics = bag.topic_table
        # print("Bagfile topics:\n", topics)

        # --- Read the desired topic ---
        for topic in topics["Topics"]:
            data = bag.message_by_topic(topic)

            # Once opened, data is saved automatically into a csv file.


def read_json(file):
    """Creates a list of experiments as objects. It then reads their respective json file and adds the metadata as
    attributes to each one of them"""

    if file.endswith(".json"):
        json_file = open(file)
        json_data = json.load(json_file)

        # Create Experiment as Object
        experiment = Experiment()

        # Add metadata as attributes
        experiment.file_source = file
        experiment.exp_type = json_data["generalInfo"]["experimentType"]
        experiment.surface = json_data["surfaceInfo"]["type"]

        experiment.x_noise = abs(json_data["robotInfo"]["x noise real [m]"])
        experiment.z_noise = abs(json_data["robotInfo"]["z noise real [m]"])

        try:
            experiment.pressure = json_data["gripperInfo"]["pressureAtValve [PSI]"]
        except KeyError:
            experiment.pressure = json_data["gripperInfo"]["pressureAtValve"]
        try:
            experiment.surface_radius = json_data["surfaceInfo"]["radius [m]"]
        except KeyError:
            experiment.surface_radius = json_data["surfaceInfo"]["radius"]

        # print(experiment.exp_type)

    return experiment


def read_csvs(experiment, folder):
    """Opens the csvs associated to each experiment and saves it as lists"""

    # Sweep the csvs of each experiment
    for file in os.listdir(folder):
        data_list = pd.read_csv(folder + "/" + file)

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

    return experiment


class Experiment():
    """Class to define experiments as objects.
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

        self.id = id

        # Data from jsons
        self.exp_type = exp_type
        self.pressure = pressure
        self.surface = surface
        self.surface_radius = radius
        self.z_noise = z_noise
        self.x_noise = x_noise
        self.time_stamp = []
        self.pressure_values = []
        self.file_source = file_source
        self.filename = ""

        # Data from csvs
        self.pressure_time_stamp = []
        self.pressure_elapsed_time = []
        self.pressure_values = []

        self.wrench_time_stamp = []
        self.wrench_elapsed_time = []
        self.wrench_zforce_values = []
        self.wrench_zforce_relative_values = []

        self.event_time_stamp = []
        self.event_elapsed_time = []
        self.event_values = []

        self.first_time_stamp = 0
        self.atmospheric_pressure = 0
        self.testrig_weight = 0
        self.errors = []

        # Statistical Features
        self.steady_pressure_values = []
        self.steady_vacuum_mean = 0
        self.steady_vacuum_std = 0
        self.max_detach_zforce = 0

    def get_atmospheric_pressure(self):
        """Takes initial and last reading as the atmospheric pressure.
        Both are taken because in some cases the valve was already on, hence the last one (after valve is off) is also checked
        """
        first_reading = self.pressure_values[0]
        last_reading = self.pressure_values[1]

        self.atmospheric_pressure = max(first_reading, last_reading)

    def get_testrig_weight(self):
        """Takes an initial reading of z-force, which is caused by the weight of the test rig"""

        self.testrig_weight = self.wrench_zforce_values[0]

    def initial_stamp(self):
        """Takes the initial stamp from all the topics. This is useful to subtract from all Time stamps and get a readable time"""
        try:
            self.first_time_stamp = min(min(self.pressure_time_stamp), min(self.wrench_time_stamp), min(self.event_time_stamp))
        except ValueError:
            self.first_time_stamp = 0

    def elapsed_times(self):
        """Subtracts the initial stamp from all the time-stamps to improve readability"""

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

        # Read the initial pressure value which is the atmospheric pressure
        self.get_atmospheric_pressure()

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
                self.steady_pressure_values.append(value - self.atmospheric_pressure)

        self.steady_vacuum_mean = np.mean(self.steady_pressure_values)
        self.steady_vacuum_std = np.std(self.steady_pressure_values)

        print("\n\nMean: %.2f and Std: %.2f" % (self.steady_vacuum_mean, self.steady_vacuum_std))

        return self.steady_vacuum_mean, self.steady_vacuum_std

    def get_relative_zforces(self):

        for i in range(len(self.wrench_zforce_values)):
            relative_force = self.wrench_zforce_values[i] - self.wrench_zforce_values[0]
            self.wrench_zforce_relative_values.append(relative_force)

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
        for time, value in zip(self.wrench_elapsed_time, self.wrench_zforce_relative_values):
            if (time > retrieve_start) and (time < vacuum_stops):
                detach_values.append(value)

        try:
            self.max_detach_zforce = max(detach_values)
        except ValueError:
            self.max_detach_zforce = "error"

        return self.max_detach_zforce

        # print("\n %.0d Maximum detachment force %.2f" % (self.id, self.max_detach_zforce))

    def check_errors(self):
        """Method to check possible errors that may invalidate the data. For instance:
        - arm didn't move and remain touching the surface after retrieve, hence no force is present.
        - suction cup collapsed in the air, and therefore showed some vacuum
        """

        # Type 1: When robot didn't move after retrieve command, then vacuum kept high right before turning valve off
        threshold = 900
        index = self.event_values.index("Vacuum Off")
        time_at_index = self.event_elapsed_time[index]
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if time > time_at_index:
                if value < threshold:
                    self.errors.append("Didn't move")
                break

        # Type 2:

    def plots_stuff(self):

        force_time = self.wrench_elapsed_time
        force_values = self.wrench_zforce_relative_values

        pressure_time = self.pressure_elapsed_time
        pressure_values = self.pressure_values

        event_x = self.event_elapsed_time
        event_y = self.event_values

        figure, axis = plt.subplots(2, 1)
        axis[0].plot(force_time, force_values)
        axis[1].plot(pressure_time, pressure_values)

        # Add vertical lines at the events
        for event, label in zip(event_x, event_y):
            axis[0].axvline(x=event, color='red', linestyle='dotted', linewidth=2)
            axis[1].axvline(x=event, color='red', linestyle='dotted', linewidth=2)
            axis[0].text(event, np.mean(force_values), label, rotation=90)
            axis[1].text(event, np.mean(pressure_values), label, rotation=90)

        # plt.ylim([0, 1200])
        axis[0].grid()
        axis[0].set_ylabel("Force [N]")
        axis[1].grid()
        axis[1].set_ylabel("Pressure [hPa]")
        plt.xlabel('elapsed time [s]')
        axis[0].set_title(self.filename)


def main():

    plt.figure()

    # exp_type = "vertical"
    exp_type = "horizontal"

    # --- Controlled variables ---
    radius = 0.0425
    # radius = 0.0375
    pressures = [50, 60, 70, 80]
    # pressures = [60, 70]
    pressures = [50]
    n_noises = 10
    n_reps = 3

    # Sweep all the pressures
    for pressure in pressures:

        list_of_means = []
        list_of_stds = []
        list_of_x_noises = []
        list_of_z_noises = []
        list_of_zforce_means = []
        list_of_zforce_stds = []

        # Sweep all the noises
        for noise in range(n_noises):

            x_noises = []
            z_noises = []
            vacuum_means = []
            vacuum_stds = []
            max_z_forces = []

            # Sweep all repetitions
            # for rep in range(n_reps):
            for rep in range(n_reps):

                # Build the name of the experiment
                location = os.path.dirname(os.getcwd())
                if exp_type == "horizontal":
                    folder = "/data/x_noise/rep" + str(rep+1) + "/"
                    filename = "horizontal_#" + str(noise) +\
                           "_pres_" + str(pressure) +\
                           "_surface_3DPrinted_with_Primer" +\
                           "_radius_" + str(radius)
                elif exp_type == "vertical":
                    folder = "/data/z_noise/rep" + str(rep + 1) + "/"
                    filename = "vertical_#" + str(noise) + \
                               "_pres_" + str(pressure) + \
                               "_surface_3DPrinted_with_Primer" + \
                               "_radius_" + str(radius)

                file_path = location + folder

                for f in os.listdir(file_path):
                    if re.match(filename, f) and f.endswith(".bag"):
                        only_filename = f.split(".bag")[0]
                        break
                    else:
                        only_filename = "no_match"
                print(only_filename)

                if only_filename == "no_match":
                    break

                # --- Step1: Turn Bag into csvs if needed ---
                # Comment if it is already done
                # bag_to_csvs(file_path + only_filename + ".bag")

                # --- Step2: Read attributes from json files
                metadata = read_json(file_path + only_filename + ".json")

                # --- Step3: Read values from the csv files for each json file
                experiment = read_csvs(metadata, (file_path + only_filename))
                experiment.filename = only_filename

                # --- Step4: Get different properties from each experiment
                # TODO Fix why is the std so small
                # TODO Finish the method to check for errors, and have the error as text in the plot
                # TODO simplify main()

                experiment.get_steady_vacuum()
                experiment.get_relative_zforces()
                max_force = experiment.get_detach_force()

                # plt.close('all')
                experiment.plots_stuff()
                plt.show()

                if max_force == "error":
                    break

                x_noise = experiment.x_noise
                z_noise = experiment.z_noise

                steady_vacuum_mean = round(experiment.steady_vacuum_mean, 2)
                steady_vacuum_std = round(experiment.steady_vacuum_std, 4)
                detach_force = round(experiment.max_detach_zforce)

                # A few exceptions due to some errors during the experiment (e.g. arm didn't move, or cup collapsed)
                if steady_vacuum_mean > -200 and z_noise < 0.02 and exp_type == "vertical":
                    print("**** There was an issue during the experiment and this point should be tossed away***")
                    print(steady_vacuum_mean, steady_vacuum_std)
                    print(x_noise, z_noise)
                    break
                if steady_vacuum_mean < -400 and x_noise > 0.027 and exp_type == "horizontal":
                    print("**** There was an issue during the experiment and this point should be tossed away***")
                    print(steady_vacuum_mean, steady_vacuum_std)
                    print(x_noise, z_noise)
                    break
                if steady_vacuum_mean > -200 and x_noise < 0.025 and exp_type == "horizontal":
                    print("**** There was an issue during the experiment and this point should be tossed away***")
                    print(steady_vacuum_mean, steady_vacuum_std)
                    print(x_noise, z_noise)
                    break

                x_noises.append(x_noise)
                z_noises.append(z_noise)
                vacuum_means.append(steady_vacuum_mean)
                vacuum_stds.append(steady_vacuum_std)
                max_z_forces.append(detach_force)

            # --- Step2: Once all values are gathered for all repetitions, obtain the mean values
            if len(vacuum_means) == 0:
                break

            final_vacuum_mean = np.mean(vacuum_means)
            final_x_noise = np.mean(x_noises)
            final_z_noise = np.mean(z_noises)
            final_zforce_mean = np.mean(max_z_forces)
            final_zforce_std = np.std(max_z_forces)

            mean_stds = 0
            for i in range(len(vacuum_stds)):
                mean_stds += vacuum_stds[i] ** 2
            final_vacuum_std = (mean_stds / len(vacuum_stds)) ** 0.5

            list_of_means.append(round(final_vacuum_mean, 2))
            list_of_stds.append(round(final_vacuum_std, 2))
            list_of_x_noises.append(round(final_x_noise, 4))
            list_of_z_noises.append(round(final_z_noise, 4))
            list_of_zforce_means.append(round(final_zforce_mean, 2))
            list_of_zforce_stds.append(round(final_zforce_std, 2))

        # check
        print("\nVacuum Means: ", list_of_means)
        print("Vacuum Stds: ", list_of_stds)
        print("x-noises: ", list_of_x_noises)
        print("zforce Means: ", list_of_zforce_means)
        print("zforce Stds: ", list_of_zforce_stds)

        if exp_type == "horizontal":
            # plt.errorbar(list_of_x_noises, list_of_means, list_of_stds, label=(str(pressure) + " PSI"))
            plt.errorbar(list_of_x_noises, list_of_zforce_means, list_of_zforce_stds, label=(str(pressure) + " PSI"))
            title = "Cartesian noise in x, for %.4f radius" % (radius)
            plt.xlabel("x-noise [m]")
        elif exp_type == "vertical":
            # plt.errorbar(list_of_z_noises, list_of_means, list_of_stds, label=(str(pressure) + " PSI"))
            plt.errorbar(list_of_z_noises, list_of_zforce_means, list_of_zforce_stds, label=(str(pressure) + " PSI"))
            title = "Cartesian noise in z, for %.4f radius" % (radius)
            plt.xlabel("z-noise [m]")

    # plt.ylabel("Vacuum [hPa]")
    # plt.ylim([-1000, 0])
    plt.ylabel("Force [N]")
    plt.ylim([-2, 7])
    plt.xlim([0, 0.030])
    plt.legend()
    plt.grid()
    plt.title(title)
    plt.show()


if __name__ == '__main__':
    main()

