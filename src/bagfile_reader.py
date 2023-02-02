import os
import csv
from matplotlib import pyplot as plt
import pandas as pd
from bagpy import bagreader
import numpy as np


def main():
    # --- Read bagfile
    location = os.path.dirname(os.getcwd())        
    foldername = "/data/"    
    filename = "vertical_#5_pres_70_surface_3DprintedPLA_radius_0.0375_noise_-3.05.bag"
    bag = bagreader(location + foldername + filename)

    topic = "/gripper/pressure"
    pressure_data = bag.message_by_topic(topic)
    pressure = pd.read_csv(pressure_data)

    # --- Extract each vector
    pressure_time_stamp = pressure.iloc[:, 0]
    elapsed = [None] * len(pressure_time_stamp)
    for i in range(len(pressure_time_stamp)):
        elapsed[i] = pressure_time_stamp[i] - pressure_time_stamp[0]

    pressure_values = pressure.iloc[:, 1]

    # --- Plot Results
    plt.plot(elapsed, pressure_values)
    plt.ylim([0, 1300])
    plt.xlabel('elapsed time [s]')
    plt.ylabel('atmospheric pressure [hPa]')
    plt.title(filename)
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()

