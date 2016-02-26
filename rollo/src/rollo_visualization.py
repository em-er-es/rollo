#!/usr/bin/env python2
# -*- coding: utf-8 -*-
## @file rollo_visualization.py
# @author Rabbia Asghar
# @author Ernest Skrzypczyk
# 
# @date 25/2/16
# 
# @brief Visualize motion capture data and EKF estimates
#
# Command prototype: <b>rosrun rollo rollo_visualization _rate:=25</b>
# \param rate Running frequency of the node <!25 [Hz]>
# \param savepath Save path of generated images <!.>
# \param type Type of saved images <!png>
# \param format Format of saved images (dim_x x dim_y) <!512>
# \param duration Duration of visualization <!0>
#

## Import basic Python libraries
from __future__ import print_function #Python2
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import time

## Import ROS libraries
import roslib
roslib.load_manifest('rollo')
import rospy

## Import messages for Rollo
## Import Pose2D
from geometry_msgs.msg import Pose2D

## Import custom messages for Rollo
# Confirm the PYTHONPATH content
# from rollo.msg import _EKF # Works

## Import EKF message
from rollo.msg import EKF # Also works!

## Import Pose2DStamped message
from rollo.msg import Pose2DStamped

''' TODO
 * ADD & FIX DOXYGEN documentation format
 * Implement proper animation for pseudo realtime display of measurements
 *! Subscribe to mocap topic (/Optitrack/ground_pose)
 * Subscribe to preprocessor topic (Pose2Dstamped)
 *! Subscribe to EKF topic
 * 
 * TODO later
 * Implement as small buffer for data as possible
 * Double check results for animation
 * Implement saving generated images to a video (code is there)
 * Implement saving generated images to a path
 * Implement a duration parameter
 * Add references from node to topics and console colours
'''

## Global variables

## Node name using console codes
NodeName = "VIS "

## Visualize rate [Hz] == [fps]
rate = 25 # 25 [Hz] = 25 [fps]
# rate = 1 # 25 [Hz] = 25 [fps]

## Message for measurement data
# MessageMeasurement = 0

## Message for EKF data
# MessageEKF = 0

## Loop counter
loopcounter = 0
markerScale = 0

## Global flags

# if (loopcounter == 0):
## Global flag 1
flagSubscriber1 = False

## Global flag 2
flagSubscriber2 = False

figure = 0

## Set pyplot to be non-interactive
# plt.ioff()

## Subscriber callback for measurement data

def subscriberCallbackMeasurement(msg):
	# rospy.loginfo("[Rollo][%s][Sub1] Message : [ %6s | %6s | %6s ]", NodeName, msg.pose2d.x, msg.pose2d.y, msg.pose2d.theta) # //DB
	# rospy.loginfo("[Rollo][%s][Sub1] Message : [ %6s | %6s | %6s ]", NodeName, msg.x, msg.y, msg.theta) # //DB
	global MessageMeasurement
	MessageMeasurement = msg
	global flagSubscriber1
	flagSubscriber1 = True

	return 0

## Subscriber callback for EKF data

def subscriberCallbackEKF(msg):
	# rospy.loginfo("[Rollo][%s][Sub2] Message : [ %6s | %6s | %6s ]", NodeName, msg.pose2d.x, msg.pose2d.y, msg.pose2d.theta) # //DB
	global MessageEKF
	MessageEKF = msg
	global flagSubscriber2
	flagSubscriber2 = True

	return 0

## Animation
# initialization function: plot the background of each frame
def init():
	Pos.set_data([], [])
	return line

# animation function.  This is called sequentially
def animate(i):
	# global Pos
	# of = np.rad2deg(MessageMeasurement.theta) - 90
	# plt.plot(MessageMeasurement.x, MessageMeasurement.y, 'b', marker=(3, 0, of), markersize = markerScale / 3)
	# of = np.rad2deg(MessageEKF.pose2d.theta) - 90
	# plt.plot(MessageEKF.pose2d.x, MessageEKF.pose2d.y, 'b', marker=(3, 0, of), markersize = markerScale / 3)
	x = np.linspace(0, 2, 1000)
	# y = np.sin(2 * np.pi * (x - 0.01 * i))
	# Pos.set_data(MessageMeasurement.x, MessageMeasurement.y)
	Pos.set_data(x, MessageMeasurement.y)
	# Pos.set_data(1*i, 2*i)
	# return line
	return


## Generate and update plot

def generatePlot():
	rospy.loginfo("[Rollo][%s][generatePlot] Init", NodeName) # //DB

	## Generate quiver plot
	# plt.quiver(MessageMeasurement.x, MessageMeasurement.y, MessageMeasurement.theta)
	# plt.quiver(MessageEKF.pose2d.x, MessageEKF.pose2d.y, MessageEKF.pose2d.theta, cmap='gray')
	of = np.rad2deg(MessageMeasurement.theta) - 90
	# Pos = plt.plot(MessageMeasurement.x, MessageMeasurement.y, 'b', marker=(3, 0, of), markersize = markerScale / 3)
	Pos, = plt.plot(MessageMeasurement.x, MessageMeasurement.y)
	# of = np.rad2deg(MessageEKF.pose2d.theta) - 90
	# PosY = plt.plot(MessageEKF.pose2d.x, MessageEKF.pose2d.y, 'b', marker=(3, 0, of), markersize = markerScale / 3)
	# if (not loopcounter > 100):
		# plt.ioff()
		# plt.show()
		# plt.hold(False)
		# time.sleep(3)
	# plt.draw()
	# global figure
	# plt.draw()

	# First set up the figure, the axis, and the plot element we want to animate
	# fig = plt.figure()
	# ax = plt.axes(xlim=(0, 2), ylim=(-2, 2))
	# line, = ax.plot([], [], lw=2)
	# call the animator.  blit=True means only re-draw the parts that have changed.
	# anim = animation.FuncAnimation(figure, animate, init_func = init, frames = rate * 10, interval = rate, blit = True)

	# save the animation as an mp4.  This requires ffmpeg or mencoder to be
	# installed.  The extra_args ensure that the x264 codec is used, so that
	# the video can be embedded in html5.  You may need to adjust this for
	# your system: for more information, see
	# http://matplotlib.sourceforge.net/api/animation_api.html
	# anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

	plt.show()
	## Reset subscriber flags
	global flagSubscriber1
	flagSubscriber1 = False
	global flagSubscriber2
	flagSubscriber2 = False

	return 0

## Node main function

def main():
	""" Node main function
	
		More details
	"""
	## Initiliaze
	global loopcounter

	### Initialize rospy
	# roscpp_initialize(sys.argv)
	rospy.init_node('rollo_visualization', anonymous=True)

	if loopcounter == 0:
		figure = plt.figure()
		# figure, ax = plt.subplots()
		plt.axis([-4, 4, -4, 4])
		plt.grid(1)
		# plt.ion()
		# plt.show()
		markerScale = 3
		# Pos, = ax.plot(0)
		# Pos, = plt.plot(0)

	## Set frequency rate for visualization node
	rosrate = rospy.Rate(rate)

	## Subscribe to EKF topic
	rospy.Subscriber('/Rollo/ekf', EKF, subscriberCallbackEKF, queue_size = 1024)

	## Subscribe to motion capture topic
	# rospy.Subscriber('/Optitrack_Rollo/ground_pose', Pose2DStamped, subscriberCallbackMeasurement, queue_size = 1024)
	rospy.Subscriber('/Optitrack_Rollo/ground_pose', Pose2D, subscriberCallbackMeasurement, queue_size = 1024)
	# rospy.Subscriber('/Rollo/pose2dstamped', Pose2DStamped, subscriberCallbackMeasurement, queue_size = 1024)

	while not rospy.is_shutdown():
		## Main loop
		rospy.loginfo("[Rollo][%s][Main] Loop: %d", NodeName, loopcounter) # //DB

		if (flagSubscriber1 == True) and (flagSubscriber2 == True):
			rospy.loginfo("[Rollo][%s][Main] Generate and update plot", NodeName) # //DB
			# generatePlot()
			Pos, = plt.plot(MessageMeasurement.x, MessageMeasurement.y)
			global animate
			# anim = animation.FuncAnimation(figure, animate, frames = 10, interval = 4, blit = True)
			anim = animation.FuncAnimation(figure, animate, frames = 100, interval = 10)
			# anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

		## Sleep to conform node frequency rate
		rosrate.sleep()

		loopcounter = loopcounter + 1
		## Main loop end

	# rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
