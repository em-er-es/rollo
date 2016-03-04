#!/bin/env python2
# -*- coding: utf-8 -*-
#%% Information
# 86805 - Software Architectures for Robotics
# Assignment 03 - Localization system for a wheeled humanoid robot
# Rabbia Asghar, Ernest Skrzypczyk
# Odometry model
# Inputs: Initial coordinates of the robot (centre position), initial orientation of the robot in radians, n_L and n_R is rotation speed [rpm] of the respectively left and right wheel
# Outputs: Final coordinates of the robot (centre position), final orientation of the robot [radians]
# Assumptions:
# 1. The robot is a rigid body
# 2. The model represents a differential drive robot
# 3. There is no S_Lip in the wheel and the surface is plane
# 4. Both wheels are turning in the forward direction

#%% Import basic libraries
from __future__ import print_function #Python2
from matplotlib import pyplot as plt

#%% Import additional libraries for a script call and parse arguments
if __name__ == '__main__':
    import os
    import argparse
    import numpy as np
    parser = argparse.ArgumentParser(prog=os.path.basename(__file__))
    #%% Movement arguments
    parser.add_argument('-nL', '--rotation-speed-left', dest='n_L', type=float, default=10.0, help='Rotation speed of the left wheel', metavar='N_L <!10>[1/min]')
    parser.add_argument('-nR', '--rotation-speed-right', dest='n_R', type=float, default=10.0, help='Rotation speed of the right wheel', metavar='N_R <!10>[1/min]')
    parser.add_argument('-Px', '--initial-position-x', dest='P_i_x', type=float, default=0.0, help='Initial position of the robot - X coordinate', metavar='P_I_X <!0>[m]')
    parser.add_argument('-Py', '--initial-position-y', dest='P_i_y', type=float, default=0.0, help='Initial position of the robot - Y coordinate', metavar='P_I_Y <!0>[m]')
    parser.add_argument('-Th', '--initial-orientation', dest='Theta_i', type=float, default=0.0, help='Initial orientation of the robot - Theta', metavar='THETA <!0>[rad]')
    parser.add_argument('-t', '--time', dest='t', type=float, default=10.0, help='Time for the movement', metavar='t <!10.0>[s]')
    parser.add_argument('-dt', '--step-time', dest='dt', type=float, default=-100.0, help='Time step for approximation for positive values and number of steps for negative', metavar='t <-100.0>[{s, 1}]')
    parser.add_argument('-rL', '--radius-wheel-left', dest='r_L', type=float, default=0.1, help='Radius of the left wheel', metavar='R_L <!0.100>[m]')
    parser.add_argument('-rR', '--radius-wheel-right', dest='r_R', type=float, default=0.1, help='Radius of the right wheel', metavar='R_R <!0.100>[m]')
    parser.add_argument('-al', '--axle-length', dest='axle_l', type=float, default=0.205, help='Distance between wheels - Length of the axle', metavar='AXLE_L <!0.205>[m]')
    #%% Additional script relevant arguments
#    parser.add_argument('-1', '-p', '--pause', dest='pause', action='store_true', help='Pause between processed images')
#    parser.add_argument('-cp', '--color-plot', dest='colorPlot', type=bool, help='Colorize generated plot')
    parser.add_argument('-gp', '--generate-plot', dest='generatePlot', action='store_true', default=0, help='Generate a plot of movement')
    parser.add_argument('-d', '--degrees', dest='degrees', action='store_true', default=0, help='Output orientation in degrees instead of radians')
#    parser.add_argument('-ia', '--invert-axes', dest='invertAxes', type=bool, default=0, help='Invert axes on distortion plots')
#    parser.add_argument('-l', '--log', dest='logFile', type=str, help='Write a log file with results', metavar='LOGFILENAME')
    parser.add_argument('-pd', '--predefined', dest='predefined', action='store_true', default=0, help='Use a predefined sets of parameters for movement simulation')
#    parser.add_argument('-si', '--show-images', dest='showImages', action='store_true', default=0, help='Show images')
#    parser.add_argument('-s', '--save-images', dest='saveImages', action='store_true', default=0, help='Save images')
#    parser.add_argument('-sp', '--save-path', dest='savePath', type=str, default='/tmp', default=0, help='Defines path for generated images, implies save images option', metavar='<SAVEPATH><!/tmp>')
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', default=0, help='Produce more verbose output')
    args = parser.parse_args() # Parse script call arguments

    #%% Assign arguments
    n_L = float(args.n_L)
    n_R = float(args.n_R)
    P_i_x = float(args.P_i_x)
    P_i_y = float(args.P_i_y)
    t = float(args.t)
    dt = float(args.dt)
    Theta_i = float(args.Theta_i)
    r_L = float(args.r_L)
    r_R = float(args.r_R)
    axle_l = float(args.axle_l)

    #%% Script arguments
#    logFile = str(args.logFile)
    degrees = bool(args.degrees)
#    colorPlot = bool(args.colorPlot)
    generatePlot = bool(args.generatePlot)
    predefined = bool(args.predefined)
#    saveImages = bool(args.saveImages)
#    savePath = str(args.savePath)
#    showImages = bool(args.showImages)
    verbose = bool(args.verbose)

#%% Function definition
def rollo_compute_position(P_i_x, P_i_y, Theta_i, n_L, n_R, t, r_L, r_R, axle_l, verbose, degrees):
    "Calculates the final position accodring to the odometry model"
    if degrees:
        Theta_i = Theta_i / 180.0 * np.pi
    S_L = t * (n_L / 60.0) * 2 * np.pi * r_L # Linear distance traveled by left wheel in meters
    S_R = t * (n_R / 60.0) * 2 * np.pi * r_R # Linear distance traveled by right wheel in meters

    Beta = (S_L - S_R) / 2.0 # Travel angle
    r  = (S_L + S_R) / 2.0 # Travel radius
    P_f_x = P_i_x + r * np.cos(Theta_i - (Beta / 2.0))
    P_f_y = P_i_y + r * np.sin(Theta_i - (Beta / 2.0))
    Theta_f = Theta_i - Beta

    if degrees:
        Theta_f = Theta_f * 180 / np.pi
    if verbose and t_s + dt == t_f:
        print('Linear distance traveled by left wheel (S_L [m]): ', S_L)
        print('Linear distance traveled by right wheel (S_R [m]): ', S_R)
        print('Travel radius [m]: ', r)
        print('Travel angle [', ThetaFormat, ']:', Beta)
    return [P_f_x, P_f_y, Theta_f]

#%% Main script
if __name__ == '__main__':
    if degrees:
        # ThetaFormat = '°'
        ThetaFormat = 'deg'
    else:
        ThetaFormat = 'rad'

    if predefined:
        P_i_x = -2; P_i_y = -2; 
        if degrees:
            Theta_i = 45
        else:
            Theta_i = np.pi / 4.0
        n_L = 6; n_R = 5; t = 1000;
        r_L = 0.1; r_R = 0.1; axle_l = 0.205;
        print('Using predefined variables set')

    if verbose:
        print('Time (t [s]): ', t)
        print('Initial position (X [m]; Y [m]): ', P_i_x, ';', P_i_y)
        print('Initial orientation (Theta [', ThetaFormat, ']): ', Theta_i)
        print('Axle length (axle_l [m]): ', axle_l)
        print('Wheel rotation speed - left (n_L [1/min]): ', n_L)
        print('Wheel rotation speed - right (n_R [1/min]): ', n_R)
        print('Wheel radius - left (r_L [m]): ', r_L)
        print('Wheel radius - right (r_R [m]): ', r_R)

    if generatePlot:
        axi = 8.0
        axd = 1
        markerScale = 20
        lineWidth = 1.6

###TODO use array instead of scalar values
    Theta_i_t = Theta_i
    P_i_x_t = P_i_x
    P_i_y_t = P_i_y

    if dt < 0:
        steps = - float(dt)
        dt = t / steps
    else:
        steps = np.ceil(t / float(dt))

    t_s = 0
    t_f = t

    if verbose:
        print('Time step (dt [s]): ', dt)
        print('Number of steps (n [1]): ', steps)

    # for i in np.arange(0, t, t / step):
    for i in np.arange(1, steps + 1, 1):
        # [P_f_x_t, P_f_y_t, Theta_f_t] = rollo_compute_position(P_i_x_t, P_i_y_t, Theta_i_t, n_L, n_R, dt, r_L, r_R, axle_l, 0, degrees)
        [P_f_x_t, P_f_y_t, Theta_f_t] = rollo_compute_position(P_i_x_t, P_i_y_t, Theta_i_t, n_L, n_R, dt, r_L, r_R, axle_l, verbose, degrees)
        print('Loop:', int(i), 'Calculated position (x [m], y [m], Theta [', ThetaFormat, ']):', P_f_x_t, P_f_y_t, Theta_f_t)
        # print(int(i))

###TODO calculate lowest and highest points for axes
        if generatePlot:
            if not degrees:
                of = np.rad2deg(Theta_i_t) - 90
            else:
                of = Theta_i_t - 90

            plt.plot(P_f_x_t, P_f_y_t, 'b', marker=(3, 0, of), markersize = markerScale / 3)

        Theta_i_t = Theta_f_t
        P_i_x_t = P_f_x_t
        P_i_y_t = P_f_y_t

        t_s = t_s + dt

    P_f_x = P_f_x_t
    P_f_y = P_f_y_t
    Theta_f = Theta_f_t

    if verbose:
        print('Simulation duration (t_s [s]): ', t_s)

    # [P_f_x, P_f_y, Theta_f] = rollo_compute_position(P_i_x, P_i_y, Theta_i, n_L, n_R, t, r_L, r_R, axle_l, verbose, degrees)
    print('Final position (x [m], y [m]): ', P_f_x, ',', P_f_y)
    if degrees:
        # print('Final orientation (Theta [°]): ', Theta_f % 360)
        print('Final orientation (Theta [deg]): ', Theta_f % 360)
    else:
        print('Final orientation (Theta [rad]): ', Theta_f % (2 * np.pi))

    if generatePlot:
        ###TODO take the highest or lowest values from simulation and add a margin to the plot
        axx = np.max([np.ceil(np.abs(P_f_x - P_i_x) / axi) * axi, np.abs(P_f_x) + axd, np.abs(P_i_x) + axd])
        axy = np.max([np.ceil(np.abs(P_f_y - P_i_y) / axi) * axi, np.abs(P_f_y) + axd, np.abs(P_i_y) + axd])
        # plt.axis([-axx, axx, -axy, axy])
        plt.axis('equal')
        if not degrees:
            oi = np.rad2deg(Theta_i) - 90
            of = np.rad2deg(Theta_f) - 90
        else:
            oi = Theta_i - 90
            of = Theta_f - 90

        plt.plot(P_i_x, P_i_y, 'g', marker = (3, 0, oi), markersize = markerScale, linewidth = lineWidth)
        # plt.plot(P_f_x_t, P_f_y_t, 'r', marker = (3, 0, of), markersize = markerScale, linewidth = lineWidth)
        plt.plot(P_f_x, P_f_y, 'r', marker = (3, 0, of), markersize = markerScale, linewidth = lineWidth)
        plt.grid(1)
        figure = plt.gcf()
        figure.canvas.set_window_title('Rollo - Odometry model')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.tight_layout()
        plt.show()
        # print(P_f_x_t, P_f_y_t, Theta_f_t, z)
        # print(t, rollo_compute_position(P_i_x, P_i_y, Theta_i, n_L, n_R, t, r_L, r_R, axle_l, 0, degrees))
        # print(t_s, rollo_compute_position(P_i_x, P_i_y, Theta_i, n_L, n_R, z-dt, r_L, r_R, axle_l, 0, degrees))
