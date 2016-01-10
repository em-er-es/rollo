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
    parser.add_argument('-t', '--time', dest='t', type=float, default=1.0, help='Time for the movement', metavar='t <!0>[s]')
    parser.add_argument('-rL', '--radius-wheel-left', dest='r_L', type=float, default=0.1, help='Radius of the left wheel', metavar='R_L <!0.100>[m]')
    parser.add_argument('-rR', '--radius-wheel-right', dest='r_R', type=float, default=0.1, help='Radius of the right wheel', metavar='R_R <!0.100>[m]')
    parser.add_argument('-al', '--axle-length', dest='axle_l', type=float, default=5.0, help='Distance between wheels - Length of the axle', metavar='AXLE_L <!0.30>[m]')
    #%% Additional script relevant arguments
    parser.add_argument('-1', '-p', '--pause', dest='pause', action='store_true', help='Pause between processed images')
    parser.add_argument('-cp', '--color-plots', dest='colorPlots', type=bool, help='Colorize distortion plots')
    parser.add_argument('-d', '--degrees', dest='degrees', action='store_true', help='Output orientation in degrees instead of radians')
    parser.add_argument('-ia', '--invert-axes', dest='invertAxes', type=bool, help='Invert axes on distortion plots')
    parser.add_argument('-l', '--log', dest='logFile', type=str, help='Write a log file with results', metavar='LOGFILENAME')
    parser.add_argument('-pd', '--predefined', dest='predefined', type=str, help='Use a predefined sets of parameters for movement simulation', metavar='PREDEFINED')
    parser.add_argument('-si', '--show-images', dest='showImages', action='store_true', help='Show images')
    parser.add_argument('-s', '--save-images', dest='saveImages', action='store_true', help='Save images')
    parser.add_argument('-sp', '--save-path', dest='savePath', type=str, default='/tmp', help='Defines path for generated images, implies save images option', metavar='<SAVEPATH><!/tmp>')
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help='Produce more verbose output')
    args = parser.parse_args() # Parse script call arguments
    #%% Assign arguments
    n_L = float(args.n_L)
    n_R = float(args.n_R)
    P_i_x = float(args.P_i_x)
    P_i_y = float(args.P_i_y)
    t = float(args.t)
    Theta_i = float(args.Theta_i)
    r_L = float(args.r_L)
    r_R = float(args.r_R)
    axle_l = float(args.axle_l)
    #%% Script arguments
    logFile = str(args.logFile)
    degrees = bool(args.degrees)
    colorPlots = bool(args.colorPlots)
    predefined = bool(args.predefined)
    saveImages = bool(args.saveImages)
    savePath = str(args.savePath)
    showImages = bool(args.showImages)
    verbose = bool(args.verbose)

#%% Function definition
def rollo_compute_position(P_i_x, P_i_y, Theta_i, n_L, n_R, t, r_L, r_R, axle_l, verbose, degrees):
    "Calculates the final position accodring to the odometry model"
    if degrees:
        Theta_i = Theta_i / 180.0 * np.pi
    S_L = t * (n_L / 60.0) * 2 * np.pi * r_L # Linear distance traveled by left wheel in meters
    S_R = t * (n_R / 60.0) * 2 * np.pi * r_R # Linear distance traveled by right wheel in meters
    if verbose:
        print('Linear distance traveled by left wheel (S_L [m]): ', S_L)
        print('Linear distance traveled by right wheel (S_R [m]): ', S_R)
    if (S_R > S_L):
#    if (np.abs(S_R - S_L) > EPS):
        r  = (axle_l / 2.0) * ((S_L + S_R) / (S_R - S_L)) # travel_radius
        beta = S_R / (r + axle_l / 2.0) # travel_angle
        P_f_x = P_i_x - r * (1 - np.cos(beta))
        P_f_y = P_i_y + r * np.sin(beta)
        Theta_f = beta + Theta_i;
    elif (S_R < S_L):
#    elif (np.abs(S_R - S_L) < -EPS):
        r = (axle_l / 2.0) * ((S_L + S_R) / (S_L - S_R)) # travel_radius
        beta = S_R / (r - axle_l / 2.0) # travel_angle
        P_f_x = P_i_x + r * (1 - np.cos(beta))
        P_f_y = P_i_y + r * np.sin(beta)
        Theta_f = Theta_i - beta
    elif (S_R == S_L):
#    elif (np.abs(S_R - S_L) < EPS):
        r = 0
        beta = 0
        P_f_x = P_i_x + S_L * np.cos(Theta_i)
        P_f_y = P_i_y + S_L * np.sin(Theta_i)
        Theta_f = Theta_i
    else:
        print('Error')
        exit(1);
    if degrees:
        Theta_f = Theta_f * 180 / np.pi
    if verbose:
        print('Travel radius [m]: ', r)
        if degrees:
            print('Travel angle [°]:', beta)
        else:
            print('Travel angle [rad]:', beta)
    return [P_f_x, P_f_y, Theta_f]

#%% Main script
if __name__ == '__main__':
    if verbose:
        print('Time (t [s]): ', t)
        print('Initial position (X [m]; Y [m]): ', P_i_x, ';', P_i_y)
        if degrees:
            print('Initial orientation (Theta [°]): ', Theta_i)
        else:
            print('Initial orientation (Theta [rad]): ', Theta_i)
        print('Axle length (axle_l [m]): ', axle_l)
        print('Wheel radius - left (r_L [m]): ', r_L)
        print('Wheel radius - right (r_R [m]): ', r_R)
    if predefined:#(P_i_x, P_i_y, Theta_i, n_L, n_R, t, r_L, r_R, axle_l, verbose, degrees)
        [P_f_x, P_f_y, Theta_f] = rollo_compute_position(0, 0, 0, 10, 10, 10, 0.1, 0.1, 0.3, 1, 1)
    else:
        [P_f_x, P_f_y, Theta_f] = rollo_compute_position(P_i_x, P_i_y, Theta_i, n_L, n_R, t, r_L, r_R, axle_l, verbose, degrees)
    print('Final position (X [m]; Y [m]): ', P_f_x, ';', P_f_y)
    print('Final orientation (Theta [rad]): ', Theta_f)
