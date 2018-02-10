import numpy as np
import sys
import pdb
import math

from matplotlib import pyplot as plt

def draw_robot(points):

    fig = plt.figure(1)
    angle = points[:,2]

    startx = points[:, 0] / 10
    starty = points[:, 1] / 10

    endx, endy = draw_pointer_end(startx, starty, angle)

    plt.plot(startx, starty, 'r.')
    plt.plot([startx, endx], [starty, endy], 'g-')

def draw_pointer_end(startx, starty, angle):

    length = 30

    endy = starty + length * np.array([math.sin(x) for x in angle])
    endx = startx + length * np.array([math.cos(x) for x in angle])

    return endx, endy

def draw_laser_beam(tracings, robo_loc):

    n = tracings.size
    startx = robo_loc[0]/10
    starty = robo_loc[1]/10
    robo_ori = robo_loc[2]

    for i in range(n):
        length = tracings[i]
        cur_angle = i*math.pi/n

        endx = startx + length * math.cos(robo_ori + cur_angle - math.pi/2)
        endy = starty + length * math.sin(robo_ori + cur_angle - math.pi/2)

        fig = plt.figure(1)
        plt.plot([startx, endx], [starty, endy], 'b-')

