""" Handy to homogenize filenames"""
import os

# Paste files location
location = '/home/alejo/gripper_ws/src/suction-experiment/data'

reps = 1

for i in range(reps):

    # folder = "/data/DATASET2/x_noise/rep" + str(i + 1) + "/"
    folder = '/'

    for f in os.listdir(location + folder):

        old_text = 'TRIAL1'
        new_text = 'Gloss_Fake_cupA'

        if old_text in f:
            old = f
            new = old.replace(old_text, new_text)
            os.rename(location + folder + old, location + folder + new)
            print(old)
            print(new)