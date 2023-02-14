import os

location = os.path.dirname(os.getcwd())

for i in range(3):

    folder = "/data/x_noise/rep" + str(i + 1) + "/"

    for f in os.listdir(location + folder):
        if "printed" in f:
            old = f
            new = old.replace("printed", "Printed")
            os.rename(location + folder + old, location + folder + new)
            print(old)
            print(new)