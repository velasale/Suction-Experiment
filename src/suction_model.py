from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle, Circle
import math
import plotly.express as px
import numpy as np

class Apple():
    def __init__(self):
        self.diameter = 85
        self.center_loc = [0, 0]

class SuctionCup():
    def __init__(self):
        self.compliant_height = 12
        self.compliant_diameter = 21
        self.rigid_diameter = 23
        self.rigid_height = 18
        self.angle = 0
        self.location = [0, -50]

def get_offset(radius, distance, theta):
    """Given a certain distance between the center of the apple and the edge of the cup,
    and a rotation angle of the suction cup, calculate the offset between the center of
    the apple and the centerline of the cup"""

    beta = math.asin(distance / radius)
    angle = beta + math.radians(theta)
    offset = abs(radius * math.sin(angle))

    return offset



def data_from_exp():

    # ------ Vertical Experiments -------
    FeedIn Pressure: 40
    zNoises: [0.0, 0.0008, 0.0022, 0.0031, 0.0042, 0.0053, 0.0065, 0.0075, 0.0086, 0.0097, 0.0108, 0.0119]
    zForce means: [4.39, 4.29, 4.47, 4.51, 4.48, 4.23, 4.09, 4.22, 4.31, 4.32, 4.33, 0.07]

    Feed In Pressure: 50
    zNoises: [0.0001, 0.001, 0.0021, 0.0031, 0.0042, 0.0054, 0.0065, 0.0075, 0.0086, 0.0097, 0.0108, 0.0119]
    zForce means: [4.54, 4.44, 4.32, 4.22, 4.3, 4.76, 4.6, 4.18, 4.12, 4.34, 4.33, 0.18]

    FeedIn
    Pressure: 60
    zNoises: [0.0, 0.0009, 0.0021, 0.0031, 0.0043, 0.0053, 0.0064, 0.0075, 0.0086, 0.0097, 0.0108, 0.0119]
    zForce means: [4.72, 4.3, 4.33, 4.19, 4.46, 4.93, 4.12, 4.02, 4.28, 4.24, 4.17, 0.07]

    Feed InPressure: 70
    zNoises: [0.0001, 0.0009, 0.0021, 0.0032, 0.0043, 0.0054, 0.0065, 0.0075, 0.0086, 0.0097, 0.0108, 0.0118]
    zForce means: [4.94, 4.56, 4.18, 4.1, 4.29, 4.19, 4.1, 3.96, 4.24, 4.32, 4.24, 0.04]

    # --------- HORIZONTAL EXPERIMENTS ---------
    Feed In Pressure: 40
    zForce means: [4.61, 4.32, 4.13, 3.86, 3.72, 3.65, 3.31, 3.13, 0.13, -0.13]
    xForce means: [0.29, 0.57, 1.15, 1.83, 2.2, 2.57, 2.47, 2.36, 0.12, 0.04]
    xNoises: [0.0001, 0.0041, 0.0083, 0.0124, 0.0167, 0.0207, 0.0248, 0.0291, 0.0332, 0.0374]

    Feed In Pressure: 50
    zForce means: [4.8, 4.71, 4.29, 4.0, 3.87, 3.71, 3.11, 3.24, 0.25, -0.12]
    xForce means: [0.2, 0.66, 1.34, 1.74, 2.13, 2.62, 2.53, 2.34, 0.19, 0.25]
    xNoises: [0.0, 0.004, 0.0083, 0.0124, 0.0166, 0.0208, 0.0249, 0.0291, 0.0332, 0.0374]

    Feed In Pressure: 60
    zForce means: [5.12, 4.71, 4.54, 4.13, 3.94, 3.97, 3.37, 3.21, 0.07, -0.07]
    xForce means: [0.22, 0.51, 1.3, 1.72, 2.19, 2.68, 2.54, 2.21, 0.26, 0.26]
    xNoises: [0.0001, 0.0041, 0.0084, 0.0125, 0.0165, 0.0207, 0.0249, 0.0291, 0.0332, 0.0373]

    Feed In Pressure: 70
    zForce means: [5.19, 4.77, 4.36, 4.03, 3.98, 3.82, 3.41, 3.3, 0.04, -0.2]
    xForce means: [0.2, 0.68, 1.31, 1.72, 2.49, 2.75, 2.62, 2.26, 0.24, -0.05]
    xNoises: [0.0, 0.0041, 0.0083, 0.0125, 0.0166, 0.0208, 0.0249, 0.0291, 0.0332, 0.0374]


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

    #85cm data @ 60PSI
    x_noises = [0.0001, 0.0041, 0.0084, 0.0125, 0.0165, 0.0207, 0.0249, 0.0291, 0.0332, 0.0373]
    zforce_means = [5.12, 4.71, 4.54, 4.13, 3.94, 3.97, 3.37, 3.21, 0.07, -0.07]

    # x_noises = [0.0, 0.0031, 0.0063, 0.0097, 0.0127, 0.0159, 0.0192, 0.0224, 0.0255, 0.0287]
    # zforce_means = [4.72, 4.34, 4.42, 4.11, 3.75, 3.67, 3.46, 3.21, 3.15, 0.14]

    #75cm data
    # x_noises = [0.0, 0.0026, 0.0053, 0.0081, 0.0108, 0.0135, 0.0162, 0.0188, 0.0216, 0.0243]
    # z_force_means = [4.85, 4.66, 4.19, 4.16, 4.19, 3.8, 3.63, 3.39, 3.18, 3.27]

    apple = Apple()
    leftsc = SuctionCup()
    rightsc = SuctionCup()

    map_forces = []
    for distance in range(0, 35, 1):
        thetas = []
        offsets = []
        zforces_interp = []
        thetas_plot = []
        for theta in range(-80, 60, 1):
            thetas_plot.append(theta)
            offset = get_offset(apple.diameter/2, distance, theta)
            if (abs(offset) + leftsc.compliant_diameter/2) < apple.diameter/2:
                thetas.append(round(theta, 1))
                offsets.append(round(offset, 1))

                # Interpolate to obtain zforce
                index=1
                for baba in x_noises:
                    if baba > offset:
                        index = x_noises.index(baba)
                    break

                x1 = x_noises[index]*1000
                x0 = x_noises[index-1]*1000
                y1 = zforce_means[index]
                y0 = zforce_means[index-1]

                y = y1 + (y0-y1)*(x1 - offset)/(x1-x0)
                zforces_interp.append(round(y, 2))

            else:
                thetas.append('Nan')
                offsets.append('Nan')
                zforces_interp.append('Nan')

        print('\nDistance:', distance)
        print(thetas)
        print(offsets)
        print(zforces_interp)

        map_forces.append(zforces_interp)

    print(len(zforces_interp), len(thetas_plot))

    new = np.transpose(map_forces)
    fig = px.imshow(new, y=thetas_plot, aspect='auto')
    fig.update_traces(colorbar_orientation='h', selector=dict(type='heatmap'))
    fig.show()

if __name__ == '__main__':
    main()

