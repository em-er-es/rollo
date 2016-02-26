#!/bin/env python2
# -*- coding: utf-8 -*-
#%% Information
# 86805 - Software Architectures for Robotics
# Assignment 03 - Localization system for a wheeled humanoid robot
# Rabbia Asghar, Ernest Skrzypczyk
# EKF - Extended Kalman Filter for nonlinear dynamic systems
# Based upon original Matlab code by Yi Cao from Cranfield University, 02/01/2008
# Modified by Rabbia Asghar specifically for Rollo odometry

#%% Description
# [x, P] = rollo_ekf(f, x, P, h, z, Q, R)
# Returns state estimate x and state covariance P
# For nonlinear dynamic system:
#           x_k+1 = f(x_k, u_k) + w_k
#           z_k   = h(x_k) + v_k
# where w ~ N(0, Q) is gaussian noise with zero average and covariance Q
#       v ~ N(0, R) is gaussian noise with zero average and covariance R
# Inputs:   f: Function handle for f(x)
#           x: "A priori" state estimate
#           P: "A priori" estimated state covariance
#           h: Function handle for h(x)
#           z: Current measurement
#           Q: Process noise covariance
#           R: Measurement noise covariance
# Output:   x: "A posteriori" state estimate
#           P: "A posteriori" state covariance

# [x, P] = ekf(fstate, x, u, Jf, Jh, P, hmeas, z, Q, R)
# Returns state estimate x and state covariance P
# For nonlinear dynamic system:
#           x_k = f(x_k-1, u_k-1) + w_k
#           z_k = h(x_k) + v_k
# where w ~ N(0, Q) is gaussian noise with zero average and covariance Q
#       v ~ N(0, R) is gaussian noise with zero average and covariance R
# Inputs:   fstate: function handle for f(x_k-1, u_k-1)
#           x_pp: "A priori" state estimate, x_k-1|k-1 (p for previous refers to k-1)
#           u: Control input, u_k-1
#           Jfunc: Function handle for jacobian matrix with partial derivatives of f(x_k-1, u_k-1)
#           Jh: Jacobian matrix with partial derivatives of h(x_k)
#           E_pp: "A priori" estimated state covariance, E_k-1|k-1 (p for previous refers to k-1)
#           hmeas: Function handle for h(x)
#           z: Current measurement
#           Q: Process noise covariance
#           R: Measurement noise covariance
# Outputs:  x_cc:   "A posteriori" state estimate, x_k|k (c for current refers to k)
#           E_cc:   "A posteriori" state covariance, E_k|k (c for current refers to k)


#%% Import basic libraries
from __future__ import print_function #Python2
from matplotlib import pyplot as plt

#%% Import additional libraries for a script call and parse arguments
if __name__ == '__main__':
    import os
    import argparse
    import numpy as np
    import csv
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
    parser.add_argument('-cp', '--color-plots', dest='colorPlots', help='Colorize distortion plots')
    parser.add_argument('-d', '--degrees', dest='degrees', action='store_true', help='Output orientation in degrees instead of radians')
    # parser.add_argument('-ia', '--invert-axes', dest='invertAxes', help='Invert axes on distortion plots')
    # parser.add_argument('-l', '--log', dest='logFile', type=str, help='Write a log file with results', metavar='LOGFILENAME')
    parser.add_argument('-pd', '--predefined', dest='predefined', help='Use a predefined sets of parameters for movement simulation', metavar='PREDEFINED')
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
def rollo_ekf(fstate, x_pp, u, Jfunc, Jh, E_pp, hmeas, z, Q, R):
    "Calculates the final position accodrding to the odometry model of Rollo"

#function [x_cc, E_cc] = rollo_ekf(fstate, x_pp, u, Jfunc, Jh, E_pp, hmeas, z, Q, R)

#%% PREDICTION UPDATE
# Nonlinear update and linearization at current state
    x_cp = fstate(x_pp, u) # state prediction, x_k|k-1
    Jf = Jfunc(x_pp, u)

# Partial covariance update
    E_cp = Jf * E_pp * np.transpose(Jf) + Q # E_k|k-1

#%% INNOVATION UPDATE
# Nonlinear measurement and linearization
    z_estimate = hmeas(x_cp);

    P12 = E_cp * np.transpose(Jh) # Cross covariance
    S_inv = np.inv(Jh * P12 + R);
    # M_inv = matrix3by3_inverse(H*P12+R)
    M_inv = np.invert(H * P12 + R)
    H = P12 * S_inv # Kalman filter gain, H_k

    x_cc = x_cp + H * (z - z_estimate) # State estimate, x_k|k
    E_cc = E_cp - H * np.transpose(P12) # State covariance matrix, E_k|k

#    if degrees:
#        Theta_i = Theta_i / 180.0 * np.pi
#    S_L = t * (n_L / 60.0) * 2 * np.pi * r_L # Linear distance traveled by left wheel in meters
#    S_R = t * (n_R / 60.0) * 2 * np.pi * r_R # Linear distance traveled by right wheel in meters
#    if verbose:
#        print('Linear distance traveled by left wheel (S_L [m]): ', S_L)
#        print('Linear distance traveled by right wheel (S_R [m]): ', S_R)
#    if (S_R > S_L):
##    if (np.abs(S_R - S_L) > EPS):
#        r  = (axle_l / 2.0) * ((S_L + S_R) / (S_R - S_L)) # travel_radius
#        beta = S_R / (r + axle_l / 2.0) # travel_angle
#        P_f_x = P_i_x - r * (1 - np.cos(beta))
#        P_f_y = P_i_y + r * np.sin(beta)
#        Theta_f = beta + Theta_i;
#    elif (S_R < S_L):
##    elif (np.abs(S_R - S_L) < -EPS):
#        r = (axle_l / 2.0) * ((S_L + S_R) / (S_L - S_R)) # travel_radius
#        beta = S_R / (r - axle_l / 2.0) # travel_angle
#        P_f_x = P_i_x + r * (1 - np.cos(beta))
#        P_f_y = P_i_y + r * np.sin(beta)
#        Theta_f = Theta_i - beta
#    elif (S_R == S_L):
##    elif (np.abs(S_R - S_L) < EPS):
#        r = 0
#        beta = 0
#        P_f_x = P_i_x + S_L * np.cos(Theta_i)
#        P_f_y = P_i_y + S_L * np.sin(Theta_i)
#        Theta_f = Theta_i
#    else:
#        print('Error')
#        exit(1);
#    if degrees:
#        Theta_f = Theta_f * 180 / np.pi
#    if verbose:
#        print('Travel radius [m]: ', r)
#        if degrees:
#            print('Travel angle [°]:', beta)
#        else:
#            print('Travel angle [rad]:', beta)
    return [x_cc, E_cc]

#%% Main script
if __name__ == '__main__':
	print('Extended Kalman Filter - Rollo:\n')
    if verbose:
        print('Verbose info:\n')

#%% Description
# [x, P] = rollo_ekf(f, x, P, h, z, Q, R)
# Returns state estimate x and state covariance P
# For nonlinear dynamic system:
#           x_k+1 = f(x_k, u_k) + w_k
#           z_k   = h(x_k) + v_k
# where w ~ N(0, Q) is gaussian noise with covariance Q and average 0
#       v ~ N(0, R) is gaussian noise with covariance R and average 0
# Inputs:   f: Function handle for f(x)
#           x: "A priori" state estimate
#           P: "A priori" estimated state covariance
#           h: Function handle for h(x)
#           z: Current measurement
#           Q: Process noise covariance
#           R: Measurement noise covariance
# Output:   x: "A posteriori" state estimate
#           P: "A posteriori" state covariance

#%% Initialization of the system
n = 3 # Number of states
#Q needed to be modeled in actual system
q = 0.1 # Standard deviation of process noise
r = 0.1 # Standard deviation of measurement noise
Q = q**2 * np.eye(n) # Covariance of process
R = r**2 * np.eye(n) # Covariance of measurement

#%% Input from logfile

#[x_log, y_log, theta_log] = extract_position_edit2('Test1v6AB.log', 1000, 1500, 1);
#x_log = np.array()
#x_log = np.empty(np.zeros(1, ), dtype = np.float64)
#x_log = np.empty(np.zeros(1, ), dtype = np.float64)
#x_log = np.array([], dtype = np.float64)
#x_log = []

buf = 1024 # Samples to process at a time; Frequency of samples from motion capture can reach 120 [fps]

# Initialization of log variables
x_log = np.zeros(buf, dtype = np.float64)
y_log = np.zeros(buf, dtype = np.float64)
theta_log = np.zeros(buf, dtype = np.float64)

i = 0
with open('input.log', 'r') as roslogfile:
    logreader = csv.reader(roslogfile, delimiter = ':')
    for line in logreader:
        if not i % buf:
#            print("Size pre", np.size(x_log)) #DB
            x_log = np.append(x_log, np.zeros(buf))
            y_log = np.append(y_log, np.zeros(buf))
            theta_log = np.append(theta_log, np.zeros(buf))
#            print("Size post", np.size(x_log)) #DB
#        print("L", i, ":", line) #DB
        if line[0] == "x":
            x_log[i] = line[1]
            i = i + 1
        if line[0] == "y":
            x_log[i] = line[1]
            i = i + 1
        if line[0] == "theta":
            x_log[i] = line[1]
            i = i + 1
# """
print("Size pre x", np.size(x_log)) #DB
x_log = np.trim_zeros(x_log, 'b')
print("Size post x", np.size(x_log)) #DB
print("Size pre y", np.size(y_log)) #DB
y_log = np.trim_zeros(x_log, 'b')
print("Size post y", np.size(y_log)) #DB
print("Size pre theta", np.size(theta_log)) #DB
theta_log = np.trim_zeros(x_log, 'b')
print("Size post theta", np.size(theta_log)) #DB
# """
#print("Parsing of log complete. ", i , " Lines read.") #VB
for i in np.arange(0, x_log.size - 1, x_log.size / buf): #DB
    print("x:", x_log[i], "y:", y_log[i], "theta:", theta_log[i]) #DB

# Assuming straight line motion
deltaS = ( x_log[-1] - x_log[0] ) / float(x_log.size);
u = np.array([[deltaS], [0]]) # Control input [delta S, theta]; Assumed here to be constant
s = np.array([[x_log[1]], [y_log[1]], [theta_log[1]]]) # Initial state
x = s + q * np.random.randn(3, 1); # Initial state with noise; Noise is optional as inital state is accurately read from motion capture

f=@(x,u)[x(1) + u(1)*cos(x(3) - (u(2)/2)); x(2) + u(1)*sin(x(3) - (u(2)/2)); x(3) - u(2)] # Nonlinear state equations, f(x_k-1, u_k-1)
h=@(x)[x(1); x(2); x(3)] # Measurement equation, h(x_k)

#Jf =@ (x,u) [1, 0, - u(1)*sin(x(3) - (u(2)/2)); 0, 1, u(1)*cos(x(3) - (u(2)/2)); 0, 0, 1] # Jacobian matrix with the partial derivatives of f(x_k-1, u_k-1) w.r.t x
Jh = np.eye(n) # Jacobian matrix with the partial derivatives of h(x_k) w.r.t x; Assumed here to be identity

E = np.eye(n) # Initial state covariance
N=100 # Total dynamic steps
xV = np.zeros(n, N) # Allocate memory for state
zV = np.zeros(n, N) # Allocate memory for measurement

#for k=1:N
#  z = [x_log(k+1);y_log(k+1);theta_log(k+1)];      % measurments from sensor
#  zV(:,k)  = z;                             % save measurment
#  [x, E] = rollo_ekf(f,x,u,Jf,Jh,E,h,z,Q,R);    % EKF
#  xV(:,k) = x;                            % save estimate
#end
#figure
#
# Plot results
#for k=1:3
#  subplot(3,1,k)
#  plot(1:N, xV(k,:), '-', 1:N, zV(k,:), '--')
#  title(['x state [',num2str(k),']'])
#  legend('state estimate', 'measurement')
#end
