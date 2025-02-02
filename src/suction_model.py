from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle, Circle
import math
import plotly.express as px
import numpy as np


def get_offset(radius, distance, theta):
    """Given a certain distance between the center of the apple and the edge of the cup,
    and a rotation angle of the suction cup, calculate the offset between the center of
    the apple and the centerline of the cup
    @radius: radius of the apple
    @distance: distance from the center of the apple
    @theta: tilting angle of the suction cup
    """

    if distance < radius:
        beta = math.asin(distance / radius)
        angle = beta + math.radians(theta)
        offset = abs(radius * math.sin(angle))
    else:
        offset = 1e9

    return offset


def interpolate(x, x0, x1, y0, y1, variable):
    """Linear Interpolation"""

    if y0 != 'Nan' and y1 != 'Nan':
        y = (x - x0) * (y1 - y0) / (x1 - x0) + y0
        y = round(y, 2)
    else:
        if variable == 'sumForce_means':
            y = 0
        elif variable == 'Vacuum_means':
            y = 1000

    return y


def data_from_pitch_exp(angle, diameter):
    """This data was collected in the pitch experiments"""

    # ---------------------- 75cm data ----------------------
    pitch_0_75 = {
        'xNoises': [0.0001, 0.0037, 0.0076, 0.0113, 0.0151, 0.0188, 0.0227, 0.0264, 0.0302, 0.034],
        'Vacuum_means': [216.33, 366.2, 482.86, 478.96, 459.65, 513.07, 900.36, 952.62, 954.55, 953.12],
        'sumForce_means': [5.15, 5.05, 4.54, 4.24, 4.37, 4.46, 1.56, 1.36, 1.12, 1.04]
    }

    pitch_15_75 = {
        'xNoises': [0.0001, 0.0038, 0.0075, 0.0113, 0.0151, 0.0189, 0.0227, 0.0265, 0.0303, 0.034],
        'Vacuum_means': [398.78, 307.31, 261.95, 454.53, 534.73, 568.12, 586.6, 611.91, 780.81, 953.11],
        'sumForce_means': [5.03, 5.11, 5.38, 4.53, 4.24, 3.92, 3.79, 3.5, 2.37, 0.87]
    }

    pitch_30_75 = {
        'xNoises': [0.0001, 0.0038, 0.0077, 0.0114, 0.0152, 0.0189, 0.0228, 0.0266, 0.0303, 0.0341],
        'Vacuum_means': ['Nan', 518.46, 495.3, 544.53, 624.27, 654.57, 678.84, 643.12, 662.88, 952.83],
        'sumForce_means': ['Nan', 5.94, 4.6, 4.46, 4.57, 3.66, 4.08, 4.03, 4.12, 0.41]
    }

    pitch_45_75 = {
        'xNoises': [0.0001, 0.0038, 0.0077, 0.0114, 0.0151, 0.0189, 0.0227, 0.0265, 0.0302, 0.034],
        'Vacuum_means': ['Nan', 'Nan', 'Nan', 'Nan', 687.34, 685.49, 704.26, 731.37, 690.17, 823.69],
        'sumForce_means': ['Nan', 'Nan', 'Nan', 'Nan', 4.27, 4.29, 4.33, 4.02, 4.09, 2.52]
    }

    # ---------------------- 85cm data ----------------------
    pitch_0_85 = {
        'xNoises': [0.0001, 0.0044, 0.0089, 0.0134, 0.0179, 0.0223, 0.0268, 0.0313, 0.0358, 0.0403],
        'Vacuum_means': [255.63, 351.08, 531.52, 474.68, 476.8, 566.28, 770.67, 952.84, 953.15, 954.83],
        'sumForce_means': [5.14, 4.98, 4.47, 4.55, 4.5, 3.74, 2.59, 1.08, 1.17, 0.87]
    }

    pitch_15_85 = {
        'xNoises': [0.0001, 0.0045, 0.009, 0.0135, 0.0179, 0.0224, 0.0269, 0.0314, 0.0358, 0.0404],
        'Vacuum_means': [548.72, 410.71, 328.09, 608.84, 622.22, 534.9, 576.7, 639.64, 951.76, 953.71],
        'sumForce_means': [5.24, 4.62, 4.97, 3.75, 3.9, 4.07, 4.06, 3.46, 0.81, 0.86]
    }

    pitch_30_85 = {
        'xNoises': [0.0001, 0.0045, 0.009, 0.0134, 0.018, 0.0225, 0.027, 0.0314, 0.0359, 0.0404],
        'Vacuum_means': ['Nan', 'Nan', 550.17, 574.08, 643.5, 776.18, 718.69, 640.74, 779.09, 953.16],
        'sumForce_means': ['Nan', 'Nan', 4.67, 4.3, 3.74, 4.01, 3.39, 4.01, 2.6, 0.36]
    }

    pitch_45_85 = {
        'xNoises': [0.0001, 0.0045, 0.009, 0.0135, 0.0179, 0.0224, 0.0269, 0.0314, 0.0358, 0.0403],
        'Vacuum_means': ['Nan', 'Nan', 'Nan', 'Nan', 660.63, 662.61, 704.81, 692.18, 663.49, 952.97],
        'sumForce_means': ['Nan', 'Nan', 'Nan', 'Nan', 4.82, 4.93, 4.47, 4.27, 4.66, 0.45]
    }

    if diameter == 75:
        if 0 <= angle < 15:
            return pitch_0_75, pitch_15_75
        elif 15 <= angle < 30:
            return pitch_15_75, pitch_30_75
        elif 30 <= angle <= 45:
            return pitch_30_75, pitch_45_75
    elif diameter == 85:
        if 0 <= angle < 15:
            return pitch_0_85, pitch_15_85
        elif 15 <= angle < 30:
            return pitch_15_85, pitch_30_85
        elif 30 <= angle <= 45:
            return pitch_30_85, pitch_45_85


def main():
    """
    This snippet creates the Force Convolution HeatMap with the data
    from the suction-cup characterization experiments
    @return:
    """

    map_forces = []

    # variable = 'Vacuum_means'
    variable = 'sumForce_means'

    FONTSIZE = 24
    TICKSIZE = 22
    FIGURESIZE = (10, 8)    # width, height

    results = []
    diameters = [75, 85]

    for diameter in diameters:

        plt.figure(figsize=FIGURESIZE)
        map_forces = []

        # Sweep all possible offsets from the center
        for distance in range(0, int(diameter/2), 1):
            thetas = []
            offsets = []
            zforces_interpolated = []

            # Sweep all the possible tilting angles at each of the offset distances
            min_angle = 0
            max_angle = 45
            delta = 1
            dist = distance / 1000
            for theta in range(min_angle, max_angle, delta):

                first, second = data_from_pitch_exp(theta, diameter)
                thetas.append(round(theta, 1))
                offsets.append(round(distance, 1))

                # First Interpolation
                # Interpolate the offset from the experiment's data to obtain expected zForce
                for x in first['xNoises']:
                    if x * 1000 > distance:
                        first_index = first['xNoises'].index(x)
                        break

                x00 = first['xNoises'][first_index - 1]
                x01 = first['xNoises'][first_index]
                y00 = first[variable][first_index - 1]
                y01 = first[variable][first_index]

                y0 = interpolate(dist, x00, x01, y00, y01, variable)

                for x in second['xNoises']:
                    if x * 1000 > distance:
                        first_index = second['xNoises'].index(x)
                        break
                x10 = second['xNoises'][first_index - 1]
                x11 = second['xNoises'][first_index]
                y10 = second[variable][first_index - 1]
                y11 = second[variable][first_index]

                y1 = interpolate(dist, x10, x11, y10, y11, variable)

                # ---- 2nd interpolation: Interpolate the offset from the experiment's data to obtain expected zForce
                if 0 <= theta < 15:
                    x0 = 0
                    x1 = 15
                elif 15 <= theta < 30:
                    x0 = 15
                    x1 = 30
                elif 10 <= theta <= 45:
                    x0 = 30
                    x1 = 45

                y = interpolate(theta, x0, x1, y0, y1, variable)
                zforces_interpolated.append(y)

            map_forces.append(zforces_interpolated)

        new = np.transpose(map_forces)
        if variable == 'sumForce_means':
            plt.imshow(new, cmap='Reds', interpolation='nearest', origin='lower')
            # plt.title('mean zForce [N] heatmap for Diameter' + str(diameter) + 'mm')
        elif variable == 'Vacuum_means':
            plt.imshow(new, cmap='Blues', interpolation='nearest', origin='lower')
            plt.title('mean Vacuum [hPa] heatmap for Diameter' + str(diameter) + 'mm')

        cbar = plt.colorbar()
        cbar.set_label('Force [N]', rotation=90, fontsize=FONTSIZE)
        # https://stackoverflow.com/questions/40184696/change-fontsize-of-colorbars-in-matplotlib
        for t in cbar.ax.get_yticklabels():
            t.set_fontsize(TICKSIZE)

        plt.ylabel('Tilt angle [deg]', fontsize=FONTSIZE)
        plt.xlabel('Offset from center [mm]', fontsize=FONTSIZE)
        plt.xticks(size=TICKSIZE)
        plt.yticks(size=TICKSIZE)

        results.append(new)
        print(len(new[0]))

    first = results[0]
    second = results[1]
    rows = 46
    cols = 42

    means = np.zeros((rows, cols))

    for i in range(first.shape[0]):
        for j in range(first.shape[1]):
            try:
                val = (first[i, j] + second[i, j]) / 2
            except IndexError:
                val = (0 + second[i, j]) / 2
            means[i, j] = val

    plt.figure(figsize=FIGURESIZE)
    if variable == 'sumForce_means':
        plt.imshow(means, cmap='Reds', interpolation='nearest', origin='lower')

        # plt.title('mean zForce [N] heatmap', fontsize=FONTSIZE)
    elif variable == 'Vacuum_means':
        plt.imshow(means, cmap='Blues', interpolation='nearest', origin='lower')
        plt.title('mean Vacuum [hPa] heatmap', fontsize=FONTSIZE)

    cbar = plt.colorbar()
    cbar.set_label('Force [N]', rotation=90, fontsize=FONTSIZE)
    # https://stackoverflow.com/questions/40184696/change-fontsize-of-colorbars-in-matplotlib
    for t in cbar.ax.get_yticklabels():
        t.set_fontsize(TICKSIZE)

    plt.ylabel('Tilt angle [deg]', fontsize=FONTSIZE)
    plt.xlabel('Offset from center [mm]', fontsize=FONTSIZE)
    plt.xticks(size=TICKSIZE)
    plt.yticks(size=TICKSIZE)

    # -------- Radar Plot ----------
    radar = np.zeros((rows, cols))
    radius = 15
    for i in range(rows):
        for j in range(cols):
            accu = 0
            cnt = 0

            # For each cell perform a "radar" sum
            for k in range(i-radius, i+radius, 1):
                if k < 0 or k > (rows - 1):
                    continue

                x = i - k
                y = int(math.sqrt(radius ** 2 - x ** 2))

                min = j - y
                max = j + y

                for l in range(min, max, 1):
                    if l < 0 or l > (cols - 1):
                        continue

                    accu += means[k, l]
                    cnt += 1

            radar[i, j] = accu / cnt

    plt.figure(figsize=FIGURESIZE)
    plt.rc('font', family='serif')
    plt.imshow(radar, cmap='Reds', interpolation='nearest', origin='lower')

    cbar = plt.colorbar()
    cbar.set_label('Force [N]', rotation=90, fontsize=FONTSIZE)
    #https://stackoverflow.com/questions/40184696/change-fontsize-of-colorbars-in-matplotlib
    for t in cbar.ax.get_yticklabels():
        t.set_fontsize(TICKSIZE)

    plt.ylabel('Tilt angle [deg]', fontsize=FONTSIZE)
    plt.xlabel('Offset from center [mm]', fontsize=FONTSIZE)
    # plt.title('Radar mean-of-values within a radius of ' + str(radius), fontsize=FONTSIZE)
    plt.xticks(size=TICKSIZE)
    plt.yticks(size=TICKSIZE)

    print(radar)
    # ------- Display a cross hair (+) at the max point
    for th in range(30):
        threshold = th
        # sub_array = radar[:, threshold:cols-1]
        sub_array = radar[:, threshold]
        if variable == 'Vacuum_means':
            max_value = sub_array.min()
        elif variable == 'sumForce_means':
            max_value = sub_array.max()

        index = np.where(radar == max_value)

        # In some cases the max value is encountered at more than one place.
        # In that case just pick the first one
        a = index[0]
        b = index[1]
        if len(a) > 1:
            index = a

        print(round(max_value, 2), index)

        if th == 24:
            # This is the chosen offset
            plt.plot(b, a, marker="*", markersize=25, markeredgecolor="yellow", markerfacecolor="yellow")
        else:
            plt.plot(b, a, marker="+", markersize=15, markeredgecolor="yellow", markerfacecolor="yellow")

        plt.xticks(size=TICKSIZE)
        plt.yticks(size=TICKSIZE)
        plt.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()

