import json
import os
import re
import csv

from operator import sub, add

import bagpy
from matplotlib import pyplot as plt
import pandas as pd
from bagpy import bagreader
import numpy as np

from sklearn.metrics import r2_score


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
            experiment.wrench_xforce_values = data_list.iloc[:, 5].tolist()
            experiment.wrench_yforce_values = data_list.iloc[:, 6].tolist()
            experiment.wrench_zforce_values = data_list.iloc[:, 7].tolist()
            experiment.wrench_xtorque_values = data_list.iloc[:, 8].tolist()
            experiment.wrench_ytorque_values = data_list.iloc[:, 9].tolist()
            experiment.wrench_ztorque_values = data_list.iloc[:, 10].tolist()

        if file == "xperiment_steps.csv":
            experiment.event_time_stamp = data_list.iloc[:, 0].tolist()
            experiment.event_values = data_list.iloc[:, 1].tolist()

    return experiment


def find_file(type, radius, pressure, noise, rep):
    """"""

    # A. Build the name of the experiment
    location = os.path.dirname(os.getcwd())

    if type == "horizontal":
        folder = "/data/x_noise/rep" + str(rep + 1) + "/"
        filename = "horizontal_#" + str(noise) + \
                   "_pres_" + str(pressure) + \
                   "_surface_3DPrinted_with_Primer" + \
                   "_radius_" + str(radius)
    elif type == "vertical":
        folder = "/data/z_noise/rep" + str(rep + 1) + "/"
        filename = "vertical_#" + str(noise) + \
                   "_pres_" + str(pressure) + \
                   "_surface_3DPrinted_with_Primer" + \
                   "_radius_" + str(radius)

    file_path = location + folder

    # B. Look for the file
    for f in os.listdir(file_path):
        if re.match(filename, f) and f.endswith(".bag"):
            only_filename = f.split(".bag")[0]
            break
        else:
            only_filename = "no_match"
    print(only_filename)

    if only_filename == "no_match":
        file = "no_match"
    else:
        file = location + folder + only_filename

    return file, only_filename


def suction_plots(type, x_noises, z_noises, mean_values, std_values, pressure, radius, trends='false'):
    if type == "horizontal":
        plt.errorbar(x_noises, mean_values, std_values, label=(str(pressure) + " PSI"))

        # Trendline
        if trends == 'true':
            z = np.polyfit(x_noises, mean_values, 4)
            p = np.poly1d(z)
            plt.plot(x_noises, p(x_noises), linestyle='dashed', color='black')
            print(r2_score(mean_values, p(x_noises)))
            plt.annotate("r-squared = {:.3f}".format(r2_score(mean_values, p(x_noises))), (0.01, -200))

        # Trial of fill between
        # plt.plot(noises_xnoises, noises_vacuum_means)
        # plt.fill_between(noises_xnoises, list(map(sub, noises_vacuum_means, noises_vacuum_stds)), list(map(add,noises_vacuum_means, noises_vacuum_stds)), alpha=.5)

        title = "Cartesian noise in x, for %.2f mm diameter" % (2000*radius)
        plt.xlabel("x-noise [m]")

    elif type == "vertical":
        plt.errorbar(z_noises, mean_values, std_values, label=(str(pressure) + " PSI"))

        # Trendline
        if trends == 'true':
            z = np.polyfit(z_noises, mean_values, 4)
            p = np.poly1d(z)
            plt.plot(z_noises, p(z_noises), linestyle='dashed', color='black')
            plt.annotate("r-squared = {:.3f}".format(r2_score(mean_values, p(z_noises))),(0.01, -200))

        title = "Cartesian noise in z, for %.2f mm diameter" % (2000*radius)
        plt.xlabel("z-noise [m]")

    plt.ylabel("Vacuum [hPa]")
    plt.ylim([-1000, 0])
    # plt.ylabel("Force [N]")
    # plt.ylim([0, 7])
    plt.xlim([0, 0.030])
    plt.legend()
    plt.grid()
    plt.title(title)
    plt.show()


class Experiment:
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

        self.wrench_xforce_values = []
        self.wrench_xforce_relative_values = []
        self.wrench_yforce_values = []
        self.wrench_yforce_relative_values = []
        self.wrench_zforce_values = []
        self.wrench_zforce_relative_values = []

        self.wrench_xtorque_values = []
        self.wrench_xtorque_relative_values = []
        self.wrench_ytorque_values = []
        self.wrench_ytorque_relative_values = []
        self.wrench_ztorque_values = []
        self.wrench_ztorque_relative_values = []

        self.event_time_stamp = []
        self.event_elapsed_time = []
        self.event_values = []

        self.first_time_stamp = 0
        self.atmospheric_pressure = 0
        self.errors = []

        # Statistical Features
        self.steady_pressure_values = []
        self.steady_vacuum_mean = 0
        self.steady_vacuum_std = 0
        self.max_detach_xforce = 0
        self.max_detach_xforce_time = 0
        self.max_detach_yforce = 0
        self.max_detach_yforce_time = 0
        self.max_detach_zforce = 0
        self.max_detach_zforce_time = 0

    def get_features(self):
        """Basically run all the methods"""
        self.elapsed_times()
        self.get_atmospheric_pressure()
        self.get_steady_vacuum()
        self.get_relative_zforces()
        self.get_detach_force()
        self.check_errors()

    def initial_stamp(self):
        """Takes the initial stamp from all the topics. This is useful to subtract from all Time stamps and get a readable time"""
        try:
            self.first_time_stamp = min(min(self.pressure_time_stamp), min(self.wrench_time_stamp),
                                        min(self.event_time_stamp))
        except ValueError:
            self.first_time_stamp = 0

    def elapsed_times(self):
        """Subtracts the initial stamp from all the topics' time-stamps to improve readability"""

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

    def get_atmospheric_pressure(self):
        """Takes initial and last reading as the atmospheric pressure.
        Both are taken because in some cases the valve was already on, hence the last one (after valve is off) is also checked
        """
        first_reading = self.pressure_values[0]
        last_reading = self.pressure_values[1]

        self.atmospheric_pressure = max(first_reading, last_reading)

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
        """Method to obtain the max force during the retrieval
        """

        # Get the index at which the steady state starts and ends
        start_index = self.event_values.index('Retrieve')
        end_index = self.event_values.index("Vacuum Off")

        # Get the time at which the steady state starts and ends
        retrieve_start = self.event_elapsed_time[start_index]
        vacuum_stops = self.event_elapsed_time[end_index]

        # Get the detachment values
        x_detach_values = []
        for time, value in zip(self.wrench_elapsed_time, self.wrench_xforce_values):
            if (time > retrieve_start) and (time < vacuum_stops):
                x_detach_values.append(value)

        # Get Max values
        try:
            self.max_detach_xforce = max(x_detach_values)
            index = self.wrench_xforce_values.index(self.max_detach_xforce)
            self.max_detach_xforce_time = self.wrench_elapsed_time[index]
        except ValueError:
            self.max_detach_xforce = "error"

        y_detach_values = []

        z_detach_values = []
        for time, value in zip(self.wrench_elapsed_time, self.wrench_zforce_relative_values):
            if (time > retrieve_start) and (time < vacuum_stops):
                z_detach_values.append(value)

        # Get Max values
        try:
            self.max_detach_zforce = max(z_detach_values)
            index = self.wrench_zforce_relative_values.index(self.max_detach_zforce)
            self.max_detach_zforce_time = self.wrench_elapsed_time[index]
        except ValueError:
            self.max_detach_zforce = "error"

        return self.max_detach_zforce, self.max_detach_xforce

        # print("\n %.0d Maximum detachment force %.2f" % (self.id, self.max_detach_zforce))

    def check_errors(self):
        """Method to check possible errors that may invalidate the data. For instance:
        - arm didn't move and remain touching the surface after retrieve, hence no force is present.
        - suction cup collapsed in the air, and therefore showed some vacuum
        """

        # 1. Cases due to the arm movement solver:

        # 1.1. When arm didnt retrieve, force and vacuum remain constant before and after retrieve event. Hence you dont
        # see much change in the zforce
        force_range = 1
        p_threshold = 800
        p_range = 50
        retrieve_index = self.event_values.index("Retrieve")
        time_at_index = self.event_elapsed_time[retrieve_index]
        for time, value in zip(self.wrench_elapsed_time, self.wrench_zforce_relative_values):
            if time > time_at_index:
                force_at_retrieve = value
                break
            else:
                force_at_retrieve = 0
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if time > time_at_index:
                pressure_at_retrieve = value
                break

        vacuum_off_index = self.event_values.index("Vacuum Off")
        time_at_index = self.event_elapsed_time[vacuum_off_index]
        for time, value in zip(self.wrench_elapsed_time, self.wrench_zforce_relative_values):
            if time > time_at_index:
                force_at_vacuum_off = value
                break
            else:
                force_at_vacuum_off = 0
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if time > time_at_index:
                pressure_at_vacuum_off = value
                break

        if (abs(force_at_retrieve - force_at_vacuum_off) < force_range) and pressure_at_retrieve < p_threshold and abs(pressure_at_vacuum_off - pressure_at_retrieve) < p_range:
            print("Error ", force_at_retrieve, force_at_vacuum_off)
            self.errors.append("Arm didn't move after Retrieve")

        if self.exp_type == "vertical" and self.z_noise < 0.01 and pressure_at_retrieve > 900:
            self.errors.append("Arm didn't move after Retrieve")

        if self.exp_type == "horizontal" and self.x_noise < 0.02 and pressure_at_retrieve > 900:
            self.errors.append("Arm didn't move after Retrieve")

        # 1.2.When for some reason, one topic stopped from being recorded. Hence, the total elapsed time is different
        time_range = 1
        force_time = self.wrench_elapsed_time[-1] - self.wrench_elapsed_time[0]
        pressure_time = self.pressure_elapsed_time[-1] - self.pressure_elapsed_time[0]

        if abs(force_time - pressure_time) > time_range:
            self.errors.append("One of the topics wasn't recorded properly")

        # 2. Cases due to the suction cup:
        # 2.1. When suction cup collapses. This shows an increase in vacuum after retrieving
        pressure_range = 50
        retrieve_index = self.event_values.index("Retrieve")
        time_at_index = self.event_elapsed_time[retrieve_index]
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if time > time_at_index:
                pressure_at_retrieve = value
                break

        vacuum_off_index = self.event_values.index("Vacuum Off")
        time_at_index = self.event_elapsed_time[vacuum_off_index]
        for time, value in zip(self.pressure_elapsed_time, self.pressure_values):
            if time > time_at_index:
                pressure_at_vacuum_off = value
                break

        if (pressure_at_retrieve - pressure_at_vacuum_off) > pressure_range:
            self.errors.append("Cup collapsed after retrieve")

    def plots_stuff(self):
        """Plots wrench (forces and moments) and pressure readings"""

        force_time = self.wrench_elapsed_time
        xforce_values = self.wrench_xforce_values
        yforce_values = self.wrench_yforce_values
        zforce_values = self.wrench_zforce_relative_values

        xtorque_values = self.wrench_xtorque_values
        ytorque_values = self.wrench_ytorque_values
        ztorque_values = self.wrench_ztorque_values

        pressure_time = self.pressure_elapsed_time
        pressure_values = self.pressure_values

        event_x = self.event_elapsed_time
        event_y = self.event_values

        max_xforce_time = self.max_detach_xforce_time
        max_xforce_val = self.max_detach_xforce
        max_zforce_time = self.max_detach_zforce_time
        max_zforce_val = self.max_detach_zforce

        # --- Labels, limits and other annotations ---
        figure, axis = plt.subplots(4, 2)
        yvalues = [zforce_values, yforce_values, xforce_values, pressure_values]
        xvalues = [force_time, force_time, force_time, pressure_time]
        ylabels = ["zForce [N]", "yForce [N]", "xForce [N]", "Pressure [hPa]"]
        ylims = [[-20, 15], [-20, 15], [-20, 15], [0, 1100]]
        # yvalues = [zforce_values, xforce_values, pressure_values]
        # xvalues = [force_time, force_time, pressure_time]
        # ylabels = ["zForce [N]", "xForce [N]", "Pressure [hPa]"]
        # ylims = [[-20, 15], [-20, 15], [0, 1100]]
        for i in range(len(axis)):
            axis[i, 0].plot(xvalues[i], yvalues[i])
            axis[i, 0].axvline(x=max_xforce_time, color='red', linestyle='dashed', linewidth=1)
            axis[i, 0].axvline(x=max_zforce_time, color='blue', linestyle='dashed', linewidth=1)
            axis[i, 0].grid()
            axis[i, 0].set_ylabel(ylabels[i])
            axis[i, 0].set_ylim(ylims[i])
            # Add vertical lines at the events
            for event, label in zip(event_x, event_y):
                axis[i, 0].axvline(x=event, color='black', linestyle='dotted', linewidth=1)
                if i == (len(axis)-1):
                    axis[i, 0].text(event, 100, label, rotation=90, color='black')
                    axis[i, 0].set_xlabel("Elapsed Time [sec]")

        # ---- Max Force Annotations ---
        axis[1, 0].annotate('Max xForce', xy=(max_xforce_time, max_xforce_val),
                         xycoords='data', xytext=(max_xforce_time + 0.5, max_xforce_val + 10),
                         va='top', ha='left', arrowprops=dict(facecolor='red', shrink=0))
        axis[0, 0].annotate('Max zForce', xy=(max_zforce_time, max_zforce_val),
                         xycoords='data', xytext=(max_zforce_time + 0.5, max_zforce_val + 10),
                         va='top', ha='left', arrowprops=dict(facecolor='blue', shrink=0))

        # --- Add error in the title if there was any ---
        try:
            error_type = self.errors[0]
        except IndexError:
            error_type = "good data"
        axis[0, 0].set_title(self.filename + "\n" + error_type)

        # --- Labels, limits and other annotations ---
        yvalues = [ztorque_values, ytorque_values, xtorque_values, pressure_values]
        xvalues = [force_time, force_time, force_time, pressure_time]
        ylabels = ["zTorque [N]", "yTorque  [N]", "xTorque [N]", "Pressure [hPa]"]
        ylims = [[-0.5, 0.5], [-0.5, 0.5], [-0.5, 0.5], [0, 1100]]

        for i in range(len(axis)):
            axis[i, 1].plot(xvalues[i], yvalues[i])
            axis[i, 1].axvline(x=max_xforce_time, color='red', linestyle='dashed', linewidth=1)
            axis[i, 1].axvline(x=max_zforce_time, color='blue', linestyle='dashed', linewidth=1)
            axis[i, 1].grid()
            axis[i, 1].set_ylabel(ylabels[i])
            axis[i, 1].set_ylim(ylims[i])
            # Add vertical lines at the events
            for event, label in zip(event_x, event_y):
                axis[i, 1].axvline(x=event, color='black', linestyle='dotted', linewidth=1)
                if i == (len(axis) - 1):
                    axis[i, 1].text(event, 0, label, rotation=90, color='black')
                    axis[i, 1].set_xlabel("Elapsed Time [sec]")


def main():

    # TODO smooth wrenches v
    # TODO Plot Moments
    # TODO PLots for different surface finishes - other experiment
    # TODO Deal with the videos and images

    plt.figure()

    # exp_type = "vertical"
    exp_type = "horizontal"

    # --- Controlled variables ---
    radius = 0.0425
    # radius = 0.0375
    pressures = [50, 60, 70, 80]
    n_noises = 10
    n_reps = 3

    # --- Sweep all the pressures ---
    for pressure in pressures:

        noises_vacuum_means = []
        noises_vacuum_stds = []
        noises_xnoises = []
        noises_znoises = []
        noises_zforce_means = []
        noises_zforce_stds = []

        # --- Sweep all the noises ---
        for noise in range(n_noises):

            reps_xnoises = []
            reps_znoises = []
            reps_vacuum_means = []
            reps_vacuum_stds = []
            reps_zforce_max = []

            # --- Sweep all repetitions ---
            for rep in range(n_reps):

                # 1. Find file
                file, only_filename = find_file(exp_type, radius, pressure, noise, rep)
                if file == "no_match":
                    break

                # 2. Turn Bag into csvs if needed
                # Comment if it is already done
                # bag_to_csvs(file + ".bag")

                # 3. Read attributes from 'json' files
                metadata = read_json(file + ".json")

                # 4. Read values from 'csv' files for each 'json' file
                experiment = read_csvs(metadata, file)
                experiment.filename = only_filename

                # 5. Get different properties for each experiment
                experiment.get_features()
                # plt.close('all')
                experiment.plots_stuff()
                plt.show()

                # 6. Check if there were any errors during the experiment
                if len(experiment.errors) > 0:
                    break

                # 7. Gather features from all the repetitions of the experiment
                reps_xnoises.append(experiment.x_noise)
                reps_znoises.append(experiment.z_noise)
                reps_vacuum_means.append(round(experiment.steady_vacuum_mean, 2))
                reps_vacuum_stds.append(round(experiment.steady_vacuum_std, 4))
                reps_zforce_max.append(experiment.max_detach_zforce)

            # --- Once all values are gathered for all repetitions, obtain the mean values
            if len(reps_vacuum_means) == 0:
                break
            final_x_noise = np.mean(reps_xnoises)
            final_z_noise = np.mean(reps_znoises)
            final_vacuum_mean = np.mean(reps_vacuum_means)
            final_zforce_mean = np.mean(reps_zforce_max)
            final_zforce_std = np.std(reps_zforce_max)

            mean_stds = 0
            for i in range(len(reps_vacuum_stds)):
                mean_stds += reps_vacuum_stds[i] ** 2
            final_vacuum_std = (mean_stds / len(reps_vacuum_stds)) ** 0.5

            noises_vacuum_means.append(round(final_vacuum_mean, 2))
            noises_vacuum_stds.append(round(final_vacuum_std, 2))
            noises_xnoises.append(round(final_x_noise, 4))
            noises_znoises.append(round(final_z_noise, 4))
            noises_zforce_means.append(round(final_zforce_mean, 2))
            noises_zforce_stds.append(round(final_zforce_std, 2))

        # --- Once all values are collected for all noises, print and plot
        suction_plots(exp_type, noises_xnoises, noises_znoises, noises_vacuum_means, noises_vacuum_stds, pressure, radius, 'true')

        # plt.show()

    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()

