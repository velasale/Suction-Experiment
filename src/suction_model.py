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


def data_from_pitch_exp85(angle):

    # ---------------------- 85cm data ----------------------
    pitch_0 = {
        'xNoises': [0.0001, 0.0044, 0.0089, 0.0134, 0.0179, 0.0223, 0.0268, 0.0313, 0.0358, 0.0403],
        'Vacuum_means': [255.63, 351.08, 531.52, 474.68, 476.8, 566.28, 770.67, 952.84, 953.15, 954.83],
        'sumForce_means': [5.14, 4.98, 4.47, 4.55, 4.5, 3.74, 2.59, 1.08, 1.17, 0.87]
    }

    pitch_15 = {
        'xNoises': [0.0001, 0.0045, 0.009, 0.0135, 0.0179, 0.0224, 0.0269, 0.0314, 0.0358, 0.0404],
        'Vacuum_means': [548.72, 410.71, 328.09, 608.84, 622.22, 534.9, 576.7, 639.64, 951.76, 953.71],
        'sumForce_means': [5.24, 4.62, 4.97, 3.75, 3.9, 4.07, 4.06, 3.46, 0.81, 0.86]
    }

    pitch_30 = {
        'xNoises': [0.0001, 0.0045, 0.009, 0.0134, 0.018, 0.0225, 0.027, 0.0314, 0.0359, 0.0404],
        'Vacuum_means': ['Nan', 'Nan', 550.17, 574.08, 643.5, 776.18, 718.69, 640.74, 779.09, 953.16],
        'sumForce_means': ['Nan', 'Nan', 4.67, 4.3, 3.74, 4.01, 3.39, 4.01, 2.6, 0.36]
    }

    pitch_45 = {
        'xNoises': [0.0001, 0.0045, 0.009, 0.0135, 0.0179, 0.0224, 0.0269, 0.0314, 0.0358, 0.0403],
        'Vacuum_means': ['Nan', 'Nan', 'Nan', 'Nan', 660.63, 662.61, 704.81, 692.18, 663.49, 952.97],
        'sumForce_means': ['Nan', 'Nan', 'Nan', 'Nan', 4.82, 4.93, 4.47, 4.27, 4.66, 0.45]
    }

    if 0 <= angle < 15:
        return pitch_0, pitch_15
    elif 15 <= angle < 30:
        return pitch_15, pitch_30
    elif 30 <= angle <= 45:
        return pitch_30, pitch_45



def data_from_pitch_exp(angle):
    """This data was collected in the pitch experiments"""

    # ---------------------- 75cm data ----------------------
    pitch_0 = {
        'xNoises': [0.0001, 0.0037, 0.0076, 0.0113, 0.0151, 0.0188, 0.0227, 0.0264, 0.0302, 0.034],
        'Vacuum_means': [216.33, 366.2, 482.86, 478.96, 459.65, 513.07, 900.36, 952.62, 954.55, 953.12],
        'sumForce_means': [5.15, 5.05, 4.54, 4.24, 4.37, 4.46, 1.56, 1.36, 1.12, 1.04]
    }

    pitch_15 = {
        'xNoises': [0.0001, 0.0038, 0.0075, 0.0113, 0.0151, 0.0189, 0.0227, 0.0265, 0.0303, 0.034],
        'Vacuum_means': [398.78, 307.31, 261.95, 454.53, 534.73, 568.12, 586.6, 611.91, 780.81, 953.11],
        'sumForce_means': [5.03, 5.11, 5.38, 4.53, 4.24, 3.92, 3.79, 3.5, 2.37, 0.87]
    }

    pitch_30 = {
        'xNoises': [0.0001, 0.0038, 0.0077, 0.0114, 0.0152, 0.0189, 0.0228, 0.0266, 0.0303, 0.0341],
        'Vacuum_means': ['Nan', 518.46, 495.3, 544.53, 624.27, 654.57, 678.84, 643.12, 662.88, 952.83],
        'sumForce_means': ['Nan', 5.94, 4.6, 4.46, 4.57, 3.66, 4.08, 4.03, 4.12, 0.41]
    }

    pitch_45 = {
        'xNoises': [0.0001, 0.0038, 0.0077, 0.0114, 0.0151, 0.0189, 0.0227, 0.0265, 0.0302, 0.034],
        'Vacuum_means': ['Nan', 'Nan', 'Nan', 'Nan', 687.34, 685.49, 704.26, 731.37, 690.17, 823.69],
        'sumForce_means': ['Nan', 'Nan', 'Nan', 'Nan', 4.27, 4.29, 4.33, 4.02, 4.09, 2.52]
    }

    if 0 <= angle < 15:
        return pitch_0, pitch_15
    elif 15 <= angle < 30:
        return pitch_15, pitch_30
    elif 30 <= angle <= 45:
        return pitch_30, pitch_45


def for_later():


    fig, ax = plt.subplots()
    # --- Plot Apple
    ax.add_patch(Circle(apple.center_loc, apple.diameter / 2, color='red', fill=False))

    # --- Plot Suction Cup
    leftsc.location[1] = apple.center_loc[1] - apple.diameter / 2 - leftsc.rigid_height
    leftsc.location[0] = apple.center_loc[0]
    ax.add_patch(Rectangle(leftsc.location, leftsc.rigid_diameter, leftsc.rigid_height,
                           angle=leftsc.angle,
                           edgecolor='blue', facecolor='blue', fill=True, lw=1))
    xloc = leftsc.location[0] + (leftsc.rigid_diameter - leftsc.compliant_diameter) / 2 + \
           (leftsc.location[1] + leftsc.rigid_height) * math.sin(math.radians(leftsc.angle))

    yloc = (leftsc.location[1] + leftsc.rigid_height) * math.cos(math.radians(leftsc.angle))

    ax.add_patch(Rectangle((xloc, yloc), leftsc.compliant_diameter, leftsc.compliant_height,
                           angle=leftsc.angle,
                           edgecolor='blue', facecolor='blue', fill=False, lw=1))

    plt.xlabel("X-AXIS")
    plt.ylabel("Y-AXIS")
    plt.title("PLOT-1")
    plt.grid()
    plt.ylim([-100, 100])
    plt.xlim([-100, 100])

    plt.show()


def main():

    # TODO interpolate in x for distance, this is for each plot interpolate the forces
    # TODO watch for the indexing of the data

    map_forces = []

    # var = 'Vacuum_means'
    var = 'sumForce_means'

    # Sweep all possible offsets from the center
    for distance in range(0, 40, 1):
        thetas = []
        offsets = []
        zforces_interpolated = []
        thetas_plot = []

        # Sweep all the possible tilting angles at each of the offset distances
        min_angle = 0
        max_angle = 45
        delta = 1

        dist = distance / 1000

        for theta in range(min_angle, max_angle, delta):


            thetas_plot.append(theta)
            first, second = data_from_pitch_exp85(theta)

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

            y00 = first[var][first_index - 1]
            y01 = first[var][first_index]

            if y00 != 'Nan' and y01 != 'Nan':
                y = (dist - x00) * (y01 - y00) / (x01 - x00) + y00
            else:
                y = 'Nan'
            y0 = y

            for x in second['xNoises']:
                if x * 1000 > distance:
                    first_index = second['xNoises'].index(x)
                    break
            x10 = second['xNoises'][first_index - 1]
            x11 = second['xNoises'][first_index]

            y10 = second[var][first_index - 1]
            y11 = second[var][first_index]

            if y10 != 'Nan' and y11 != 'Nan':
                y = (dist - x10) * (y11 - y10) / (x11 - x10) + y10
            else:
                y = 'Nan'
            y1 = y

            # ---- 2nd interpolation: Interpolate the offset from the experiment's data to obtain expected zForce
            for x in first['xNoises']:
                if x * 1000 > distance:
                    first_index = first['xNoises'].index(x)
                    break
            for y in second['xNoises']:
                if y * 1000 > distance:
                    second_index = second['xNoises'].index(y)
                    break

            print('First_index: ', first_index)
            print('Second_Index: ', second_index)

            if 0 <= theta < 15:
                x0 = 0
                x1 = 15
            elif 15 <= theta < 30:
                x0 = 15
                x1 = 30
            elif 10 <= theta <= 45:
                x0 = 30
                x1 = 45

            if y0 != 'Nan' and y1 != 'Nan':
                # y = y1 + (y0-y1)*(x1 - theta)/(x1-x0)
                y = (theta - x0) * (y1 - y0) / (x1 - x0) + y0
                y = round(y, 2)
            else:
                y ='Nan'

            zforces_interpolated.append(y)

        print('\nDistance:', distance)
        print('Tilt Angles:', thetas)
        print('Resulting offsets:', offsets)
        print('Expected zForces:', zforces_interpolated)

        map_forces.append(zforces_interpolated)

    print(len(zforces_interpolated), len(thetas_plot))

    new = np.transpose(map_forces)
    fig = px.imshow(new, y=thetas_plot, color_continuous_scale='Blues', aspect='auto', interpolation='nearest')
    fig.update_traces(colorbar_orientation='v', selector=dict(type='heatmap'))
    fig.show()


if __name__ == '__main__':
    main()

