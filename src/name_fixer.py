""" Handy to homogenize filenames"""
import os

# Paste files location
location = '/home/alejo/Documents/data/DATASET5/'
reps = 1

for i in range(reps):

    folder = '/'

    for f in os.listdir(location + folder):

        old_text = 'rep_4'
        new_text = 'rep_8'

        if old_text in f:
            old = f
            print(old)
            new = old.replace(old_text, new_text)
            os.rename(location + folder + old, location + folder + new)
            print(new)