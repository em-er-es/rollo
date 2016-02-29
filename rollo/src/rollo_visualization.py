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
# Command prototype: <b>rosrun rollo rollo_visualization _rate:=25 _plotrefreshperiod:=100</b>
# \param rate Running frequency of the node <!25 [Hz]>
# \param plotrefreshperiod Plot refresh period <!100 [1]>
# \param ms Marker scale reference value <20 [1]>
# \param saveim Save path for generated images <!.>
# \param savevid Save path for generated animation video <!.>
# \param imtype Type of saved images <!png>
# \param imformat Format of saved images (dim_x x dim_y) <!512>
# \param duration Duration of visualization <!0>
#
# \warning Not all parameters and functions are currently processed


''' TODO
 * Add parameters for refreshing plot, size
 * Make marker gradual transition from previous position to current
 * for EKF message use odompose2d and ekfpose2d
 * ADD & FIX DOXYGEN documentation format
 *C Implement proper animation for pseudo realtime display of measurements
 *! Subscribe to mocap topic (/Optitrack/ground_pose)
 *! Subscribe to preprocessor topic (Pose2Dstamped)
 *! Subscribe to EKF topic
 *
 * TODO later
 * Implement dynamic server reconfiguration, use class for main()
 *! Implement as small buffer for data as possible
 * Double check results for animation
 *P Implement saving generated images to a video (code is there)
 *P Implement saving generated images to a path
 * Implement a duration parameter
 * Add references from node to topics and console colors, parse them properly (pycparse)
'''

## Import

## Import basic Python libraries
from __future__ import print_function #Python2
## Import matplotlib
import matplotlib
## Set UI specifics
matplotlib.use('GtkAgg')
## Import plot
from matplotlib import pyplot as plt
## Import animation for export to video
from matplotlib import animation
# from matplotlib import interactive
import numpy as np
import time
## Multiprocessing library for process and pipe
from multiprocessing import Process, Pipe
import gobject

## Import ROS libraries
import roslib
roslib.load_manifest('rollo')
import rospy
#from dynamic_reconfigure.server import Server as DynamicReconfigureServer # Not used right now

## Import messages for Rollo
## Import standard Pose2D
from geometry_msgs.msg import Pose2D

## Import custom messages for Rollo
# Confirm the PYTHONPATH environment variable content

## Import EKF message
from rollo.msg import EKF
# from rollo.msg import _EKF # Also works

## Import Pose2DStamped message
from rollo.msg import Pose2DStamped

# Global variables

## Node name using console codes
NodeName = "VIS "

## Visualize rate [Hz] == [fps]
rate = 25 # 25 [Hz] = 25 [fps]

## Plot refresh period [1]
plotRefreshPeriod = 100

## Loop counter
LoopCounter = 1

## Marker scale
markerScale = 20

# Plot components and parameters

## Maximal coordinates - symmetrical\n
# Negative value used to mirror the current calibration setup of motion capture system and keep sanity with adjustments to the plot\n
# Positive value used to represent the current calibration setup of motion capture system and keep sanity with adjustments to the plot
axl = 4
# axl = -4

## X axis limit
axlx = axl

## Y axis limit
axly = axl

# Global flags

## Global message flag 1 -- Motion capture data from preprocessor
flagSubscriber1 = False

## Global message flag 2 -- Extended Kalman filter estimates and odometry modeled data from EKF node
flagSubscriber2 = False


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

### Functions unused
# Animation
## Initialization function for animation
def initAnimation():
	## Plot the background of each frame
	global Pos
	Pos.set_data([], [])
	return Pos

## Animation callback function
#
# \param i Interation step
#
# Sequentially called

def animatePlot(i):
	global Pos
	# x = np.linspace(0, 2, 1000)
	# Pos.set_data(MessageMeasurement.x, MessageMeasurement.y)
	Pos.set_data(MessageMeasurement.pose2d.x, MessageMeasurement.pose2d.y)
	# Pos.set_data(2, 2)
	# Pos.set_data(x, MessageMeasurement.y)
	# Pos.set_data(1*i, 2*i)
	return Pos

## Initilize plot

def initPlot(object):
	self.axes(xlim=(-axlx, axlx), ylim=(-axly, axly))
	self.grid(1)

# def clearPlot():
	# ax = plt.axes(xlim=(-axlx, axlx), ylim=(-axly, axly))
	# plt.grid(1)


## Generate and update plot

def generatePlot(initcond):
	rospy.loginfo("[Rollo][%s][generatePlot] Init", NodeName) # //DB

	## - First loop
	if (initcond == 0):
		## Initilize plot skeleton
		# figure, axis = plt.subplots()
		# plt.axis([-axlx, axlx, -axly, axly])
		figure = plt.figure()
		ax = plt.axes(xlim=(-axlx, axlx), ylim=(-axly, axly))
		plt.grid(1)
		# plt.ion()
		# Pos, = plt.plot(MessageMeasurement.pose2d.x, MessageMeasurement.pose2d.y)
		global Pos
		Pos, = ax.plot([], [], lw = 2)
		# plt.show(figure)
		markerScale = 3
		## Animation calls
		# anim = animation.FuncAnimation(figure, animatePlot, init_func=initAnimation, frames = 25, interval = 1, blit = True)
		# anim = animation.FuncAnimation(figure, animatePlot, init_func=initAnimation, frames = 200, interval = 20, blit = True)
		# anim = animation.FuncAnimation(figure, animatePlot, init_func=initAnimation, frames = 30, interval = 1, blit = True)
		## After initilization send the condition variable
		initcond = 1

	## Generate quiver plot
	# Pos = plt.plot(MessageMeasurement.x, MessageMeasurement.y, 'b', marker=(3, 0, of), markersize = markerScale / 3)
	# Pos, = plt.plot(MessageMeasurement.x, MessageMeasurement.y)
	Pos, = plt.plot(MessageMeasurement.pose2d.x, MessageMeasurement.pose2d.y)
	plt.show(block=False)

	# First set up the figure, the axis, and the plot element we want to animate
	# fig = plt.figure()
	# ax = plt.axes(xlim=(0, 2), ylim=(-2, 2))
	# line, = ax.plot([], [], lw=2)
	# call the animator.  blit=True means only re-draw the parts that have changed.
	# anim = animation.FuncAnimation(figure, animate, init_func = init, frames = rate * 10, interval = rate, blit = True)

	# Save the animation as an mp4.  This requires ffmpeg or mencoder to be
	# installed. The extra_args ensure that the x264 codec is used, so that
	# the video can be embedded in html5.  You may need to adjust this for
	# your system: for more information, see
	# http://matplotlib.sourceforge.net/api/animation_api.html
	# anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
	# Save to pipe, read from pipe: $ mpv video
	# anim.save('video', fps=30, extra_args=['-vcodec', 'libx264'])

	# Save generated plot
	# plt.savefig('L%d.png' % LoopCounter)
	# plt.show(block=False)

	## Reset subscriber flags
	global flagSubscriber1
	flagSubscriber1 = False
	global flagSubscriber2
	flagSubscriber2 = False

	return 0

class ProcessPlotter(object):
	def __init__(self):
		# Global variables used from callback
		global samples
		samples = 2 # Sufficient to connect markers
		# Initialize samples with zeroes
		## X data from motion capture
		self.x1 = np.zeros(samples)
		## Y data from motion capture
		self.y1 = np.zeros(samples)
		## X data from odometry model
		self.x2 = np.zeros(samples)
		## Y data from odometry model
		self.y2 = np.zeros(samples)
		## X data from EKF estimates
		self.x3 = np.zeros(samples)
		## Y data from EKF estimates
		self.y3 = np.zeros(samples)

	def terminate(self):
		plt.close('all')

	def poll_draw(self):

		def call_back():
			## Local variables
			# global samples = 10
			# samples
			# ofd = - 90
			# ofd = - 135
			## Orientation of the triangular marker
			ofd = 150

			## Set plot limits
			self.ax.axis([-axlx, axlx, -axly, axly])
			## Set plot labels
			# self.xlabel('x [m]')
			# self.ylabel('y [m]')

			## Main loop
			while 1:
				## Exception handler
				if not self.pipe.poll():
					break

				## Read data from pipe
				data = self.pipe.recv()

				## Exception handler
				if data is None:
					self.terminate()
					return False

				# Proceed with processing
				else:
					# Shift last samples
					for i in range(samples - 1):
						self.x1[i + 1] = self.x1[i]
						self.y1[i + 1] = self.y1[i]
						self.x2[i + 1] = self.x2[i]
						self.y2[i + 1] = self.y2[i]
						self.x3[i + 1] = self.x3[i]
						self.y3[i + 1] = self.y3[i]

					# Assign data from pipe to local variables
					self.x1[0] = data[1]
					self.y1[0] = data[2]
					self.x2[0] = data[4]
					self.y2[0] = data[5]
					self.x3[0] = data[7]
					self.y3[0] = data[8]

					# Correct and assign orientation of marker/robot
					of1 = np.rad2deg(data[3]) + ofd
					of2 = np.rad2deg(data[6]) + ofd
					of3 = np.rad2deg(data[9]) + ofd

					# Plot n-samples
					## Red - Data from preprocessor
					self.ax.plot(self.x1, self.y1, 'r', marker = (3, 0, of1), markersize = markerScale)
					## Green - Data from odometry sent by EKF
					self.ax.plot(self.x2, self.y2, 'g', marker = (3, 0, of2), markersize = markerScale * 0.8)
					## Green - Data from EKF
					self.ax.plot(self.x3, self.y3, 'b', marker = (3, 0, of3), markersize = markerScale * 0.6)

					if data[0]: # Clear every so often
						# rospy.loginfo("[Rollo][%s][ProcessPlotter] Clear and reinitalize plot @ loop: %d", NodeName, LoopCounter) # //VB
						rospy.loginfo("[Rollo][%s][ProcessPlotter] Clear and reinitalize plot", NodeName) # //VB
						## Clear plot
						self.ax.cla()
						## Reinitialize plot
						# initPlot(self.ax)
						self.ax.grid(1)
						## Set plot labels
						# self.xlabel('x [m]')
						# self.ylabel('y [m]')
						self.ax.axis([-axlx, axlx, -axly, axly])

			self.fig.canvas.draw()
			return True

		return call_back

	def __call__(self, pipe):
		rospy.loginfo("[Rollo][%s][ProcessPlotter] Loop: %d", NodeName, LoopCounter) # //VB

		# Initialize plot skeleton
		## Data transmission pipe between processes
		self.pipe = pipe
		self.fig, self.ax = plt.subplots()
		self.fig.canvas.set_window_title('Rollo visualization node')
		plt.grid(1)
		self.gid = gobject.timeout_add(0, self.poll_draw())

		rospy.loginfo("[Rollo][%s][ProcessPlotter] Initialized", NodeName) # //VB
		plt.ioff()
		plt.show()

class MultiProcessPlot(object):
	## Initilization
	def __init__(self):
		self.plotpipe, PlotterPipe = Pipe()
		## Called process for plotting
		self.plotter = ProcessPlotter()
		## Process holder
		self.plotprocess = Process(target = self.plotter, args = (PlotterPipe, ))
		self.plotprocess.daemon = True
		self.plotprocess.start()

	## Plot function
	def plot(self, finished=False):
		send = self.plotpipe.send

		if finished:
			send(None)
		else:
			if not LoopCounter % plotRefreshPeriod:
				reset = 1
			else:
				reset = 0

			## Compose data for pipe
			data = [reset, 
					MessageMeasurement.pose2d.x, MessageMeasurement.pose2d.y, MessageMeasurement.pose2d.theta, 
					MessageEKF.odompose2d.x, MessageEKF.odompose2d.y, MessageEKF.odompose2d.theta,
					MessageEKF.ekfpose2d.x, MessageEKF.ekfpose2d.y, MessageEKF.ekfpose2d.theta]
			# print(MessageEKF.ekfpose2d.x, MessageEKF.ekfpose2d.y, MessageEKF.ekfpose2d.theta) # //VB
			# print(MessageEKF.odompose2d.x, MessageEKF.odompose2d.y, MessageEKF.odompose2d.theta) # //VB
			## Send data through pipe
			send(data)
			## Reset global flags to receive new input
			flagSubscriber1 = False
			flagSubscriber2 = False


## Node class function
def RolloVisualization():
	""" Node main function

		More details
	"""

	##! Initiliaze:
	## - Refer to global variable so that it can be changed
	global LoopCounter

	## - Initialize rospy
	# roscpp_initialize(sys.argv)
	rospy.init_node('rollo_visualization', anonymous=True)

	# Get node parameters
	global rate
	rate = float(rospy.get_param('~rate', '25'))
	global plotRefreshPeriod
	plotRefreshPeriod = float(rospy.get_param('~plotrefreshperiod', '100'))
	global markerScale
	markerScale = float(rospy.get_param('~ms', '20'))
	# saveim = string(rospy.get_param('~saveim', '.'))
	# savevid = string(rospy.get_param('~savevid', '.'))
	# imtype = string(rospy.get_param('~imtype', 'png'))
	# imformat = int(rospy.get_param('~imformat', '512'))
	# duration = float(rospy.get_param('~d', '0'))

	## Set frequency rate for visualization node
	rosrate = rospy.Rate(rate)

	## Subscribe to EKF topic
	rospy.Subscriber('/Rollo/ekf', EKF, subscriberCallbackEKF, queue_size = 1024)

	## Subscribe to motion capture topic
	# Can use both, but need another callback for that
	# rospy.Subscriber('/Optitrack_Rollo/ground_pose', Pose2D, subscriberCallbackMeasurement, queue_size = 1024)
	rospy.Subscriber('/Rollo/pose2dstamped', Pose2DStamped, subscriberCallbackMeasurement, queue_size = 1024)

	initcond = 0

	## Multiprocessing
	## Start another process for plotting
	mpp = MultiProcessPlot()
	# processPlotting = multiprocessing.Process(target = generatePlot(0), args=())
	# processPlotting = Process(target = generatePlot(0), args=())
	# processPlotting.daemon = True
	# processPlotting.start()


	while not rospy.is_shutdown():
		## Main loop
		# rospy.loginfo("[Rollo][%s][Main] Loop: %d", NodeName, LoopCounter) # //DB
		if not LoopCounter % 1000:
			rospy.loginfo("[Rollo][%s][Main] Loop: %d", NodeName, LoopCounter) # //DB

		## Plot new message set
		if (flagSubscriber1 == True) and (flagSubscriber2 == True):
			# rospy.loginfo("[Rollo][%s][Main] Generate and update plot", NodeName) # //DB
			mpp.plot()
			## Animation
			# Pos, = plt.plot(MessageMeasurement.x, MessageMeasurement.y)
			# global animatePlot
			# anim = animation.FuncAnimation(figure, animate, frames = 10, interval = 4, blit = True)
			# anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

		## Sleep to conform node frequency rate
		rosrate.sleep()

		## Update loop counter
		LoopCounter = LoopCounter + 1

		## Main loop end

	## Wait for plotprocess to finish
	mpp.plotprocess.join()
	# rospy.spin()


## Script run condition
if __name__ == '__main__':
	try:
		RolloVisualization()
	except rospy.ROSInterruptException:
		pass
