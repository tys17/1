import numpy as np
import scipy as sp
import scipy.ndimage as ndimage
import sys
import pdb
import math
import warnings
from multiprocessing import Pool, Process, Value, Array


# start at x = 410, y = 400
global occu_thresh, init_n_particles
occu_thresh = 0.1
init_n_particles = 8000

from MapReader import MapReader
from MotionModel import MotionModel
from SensorModel import SensorModel
from Resampling import Resampling
from draw_robot import draw_robot, draw_pointer_end, draw_reference

from matplotlib import pyplot as plt
from matplotlib import figure as fig
#from matplotlib.pyplot import imshow, pause

import time

class parallel_args:
    def __init__(self):
        self.num_particles = 0
        self.u_t0 = np.array([0,0,0])
        self.u_t1 = np.array([0,0,0])
        self.ranges = np.array([0,0,0])
        self.means_type = "L"


def parallel_motion_sensor_model(m, u_t0, u_t1, ranges, meas_type, sensor_model, motion_model, X_bar_new):
    """
    MOTION MODEL
    """
    if np.linalg.norm(u_t0 - u_t1) != 0:
        x_t0 = X_bar_new[:3]
        x_t1 = motion_model.update(u_t0, u_t1, x_t0)
    else:
        x_t0 = X_bar_new[:3]
        x_t1 = x_t0

    """
    SENSOR MODEL
    """
    if (meas_type == "L" ) and (np.linalg.norm(u_t0-u_t1) != 0):
        z_t = ranges
        w_t = sensor_model.beam_range_finder_model(z_t, x_t1)
        # w_t = 1/num_particles
        X_bar_new = np.hstack((x_t1, w_t))
    else:
        X_bar_new = np.hstack((x_t1, X_bar_new[3]))

    return X_bar_new

def filter_close_map(occupancy_map):
    # filter
    occupancy_map = np.logical_and(occupancy_map < occu_thresh, occupancy_map >= 0)

    # close
    # ndimage.binary_closing(occupancy_map, structure=np.ones((100,100))).astype(np.int)

    return occupancy_map


def visualize_map(occupancy_map):
    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(occupancy_map, cmap='gray'); plt.axis([0, 800, 0, 800]);
#    x = np.random.rand(4,5)
#    imshow(x)
#    pause(10);


def visualize_timestep(X_bar, tstep, occupancy_map):
    # x_locs = X_bar[:,0]/10.0
    # y_locs = X_bar[:,1]/10.0

    # endx, endy = draw_pointer_end(x_locs, y_locs, X_bar[:,2])

    fig = plt.figure(1)
    plt.clf()

    # plot map
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(occupancy_map, cmap='gray'); plt.axis([0, 800, 0, 800]);

    # plot robots
    # scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    # plt.plot(np.vstack((x_locs, endx)).transpose(), np.vstack((y_locs, endy)).transpose(), 'g-')
    # plt.pause(0.00001)
    # scat.remove()

    draw_robot(X_bar)

def init_particles_random(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    # (randomly across the map) 
    y0_vals = np.random.uniform( 0, 8000, (num_particles, 1) )
    x0_vals = np.random.uniform( 3000, 7500, (num_particles, 1) )
    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

    # initialize weights for all particles
    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles

    X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))
    
    return X_bar_init

def init_particles_freespace(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    # (in free space areas of the map)

    """
    TODO : Add your code here
    """ 

    n_candidate = init_n_particles
    X_bar_candidate = init_particles_random(n_candidate, occupancy_map)
    
    good_particles_idx = occupancy_map[(X_bar_candidate[:,1]/10).astype(int), (X_bar_candidate[:,0]/10).astype(int)].astype(bool)
    X_bar_init = X_bar_candidate[good_particles_idx]
    # X_bar_init = np.unique(X_bar_init[:,:2], axis=0)

    # X_bar_init = X_bar_candidate[good_particles_idx]

    return X_bar_init

def init_test_particle():
    # start
    return np.array([[4150,3990,math.pi*175/180, 1], [4020,4000,math.pi, 1/init_n_particles]])

    # time = 648, 37.986323
    # return np.array([[3918.78833342,  2884.12596544,     4.71621719, 1]])

def main():

    """
    Description of variables used
    u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]   
    u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
    x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
    x_t1 : particle state belief [x, y, theta] at time t [world_frame]
    X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
    z_t : array of 180 range measurements for each laser scan
    """

    """
    Initialize Parameters
    """
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata2.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    # get the free space map
    occupancy_map = filter_close_map(occupancy_map)
    # occupancy_map = np.logical_and(occupancy_map<occu_thresh, occupancy_map>=0)
    
    logfile = open(src_path_log, 'r')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    resampler = Resampling()

    num_particles = init_n_particles
#    X_bar = init_particles_random(num_particles, occupancy_map)
    X_bar = init_particles_freespace(num_particles, occupancy_map)
    X_bar = np.vstack((X_bar, init_test_particle()))
    # X_bar = init_test_particle()

    num_particles, _ = X_bar.shape

    vis_flag = 1

    """
    Monte Carlo Localization Algorithm : Main Loop
    """
    if vis_flag:
        visualize_map(occupancy_map)

    draw_robot(X_bar)
        
    first_time_idx = True
    u_t0 = np.array([0])

    ref = np.array([[4150,3990,math.pi*175/180]])

    X_bar_new = np.zeros((0, 4), dtype=np.float64)
    laser_process = True
    LASER_MOD = 3
    laser_cnt = LASER_MOD


    for time_idx, line in enumerate(logfile):

        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        time_stamp = meas_vals[-1]

        # if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure odometry measurements for now (faster debugging)
            # continue

        ranges = np.array([0])
        if (meas_type == "L"):
            odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
            ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan
            laser_cnt = (laser_cnt+1)%LASER_MOD
        if laser_cnt == 0:
            laser_process = True
        else:
            laser_process = False


        print("Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s")

        if (first_time_idx):
            u_t0 = odometry_robot
            first_time_idx = False
            continue

        # if time_idx < 1600:
        #     continue
        #
        # if time_idx == 1600:
        #     X_bar[-1, :] = np.array([4.02993436e+03, 4.67264556e+03, 1.66812442e+00, 1])
        #     visualize_timestep(X_bar, time_idx, occupancy_map)
        #     draw_reference(ref)
            # X_bar[:, 2] -= 25/180*math.pi


        u_t1 = odometry_robot

        ref[0] = motion_model.update(u_t0, u_t1, ref[0])

        for m in range(0, num_particles):

            """
            MOTION MODEL
            """
            if np.linalg.norm(u_t0 - u_t1) != 0:
                x_t0 = X_bar[m,:3]
                x_t1 = motion_model.update(u_t0, u_t1, x_t0)
            else:
                x_t0 = X_bar[m, :3]
                x_t1 = x_t0


            """
            SENSOR MODEL
            """
            if (meas_type == "L" ) and (np.linalg.norm(u_t0-u_t1) != 0) and laser_process:
                z_t = ranges
                w_t = sensor_model.beam_range_finder_model(z_t, x_t1)
                # w_t = 1/num_particles
                # X_bar_new[m,:] = np.hstack((x_t1, w_t))
                X_bar[m, :] = np.hstack((x_t1, w_t))
            else:
                X_bar[m, :] = np.hstack((x_t1, X_bar[m, 3]))
                # X_bar_new[m,:] = np.hstack((x_t1, X_bar[m,3]))

        # X_bar = X_bar_new
        moved = np.linalg.norm(u_t0-u_t1) != 0
        u_t0 = u_t1

        """
        RESAMPLING
        """

        if moved:
            if meas_type == "L" and laser_process:
                X_bar = resampler.low_variance_sampler(X_bar)
                print (X_bar.shape)
                num_particles, _ = X_bar.shape
                fig = plt.gcf()
                visualize_timestep(X_bar, time_idx, occupancy_map)
                fig1.savefig('./vid2/time_'+str(time_idx)+'.png')
                # draw_reference(ref)

if __name__=="__main__":
    warnings.simplefilter(action='ignore', category=FutureWarning)
main()