""" Handy to homogenize filenames"""
import os

location = os.path.dirname(os.getcwd())

for i in range(4):

    folder = "/data/DATASET2/x_noise/rep" + str(i + 1) + "/"

    for f in os.listdir(location + folder):
        if "Primer70_" in f:
            old = f
            # new = old.replace("Primer70", "Primer85")
            # os.rename(location + folder + old, location + folder + new)
            print(old)
            # print(new)