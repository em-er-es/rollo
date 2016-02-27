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
# \param saveim Save path for generated images <!.>
# \param savevid Save path for generated animation video <!.>
# \param type Type of saved images <!png>
# \param format Format of saved images (dim_x x dim_y) <!512>
# \param duration Duration of visualization <!0>
#


''' TODO
 * Add parameters for refreshing plot, size
 * Make marker gradual transition from previous position to current
 * Cleanup this mess
 * for EKF message use odompose2d and ekfpose2d
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


## Import basic Python libraries
from __future__ import print_function #Python2
import matplotlib
matplotlib.use('GtkAgg')
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib import interactive
import numpy as np
import time
# import multiprocessing
from multiprocessing import Process, Pipe
import gobject

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

## Global variables

## Node name using console codes
NodeName = "VIS "

## Visualize rate [Hz] == [fps]
rate = 25 # 25 [Hz] = 25 [fps]
# rate = 30 # 25 [Hz] = 25 [fps]

## Message for measurement data
# MessageMeasurement = 0

## Message for EKF data
# MessageEKF = 0

## Loop counter
loopcounter = 0
markerScale = 0
plotRefreshRate = 100

## Plot components and parameters
## Maximal coordinates - symmetrical
axl = -4
axlx = axl
axly = axl
# axlx = 2
# axly = 2

## Global flags

## Global flag 1
flagSubscriber1 = False

## Global flag 2
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

	## Initialization function for animation
	# def initAnimation():
		## Plot the background of each frame
		# global PoS
		# Pos.set_data([], [])
		# return Pos,

	## Animation callback function
	#
	# \param i Interation step
	#
	# Sequentially called
	# def animatePlot(i):
		# global Pos
		# x = np.linspace(0, 2, 1000)
		# Pos.set_data(MessageMeasurement.x, MessageMeasurement.y)
		# Pos.set_data(MessageMeasurement.pose2d.x, MessageMeasurement.pose2d.y)
		# Pos.set_data(2, 0.1 * i)
		# Pos.set_data(x, MessageMeasurement.y)
		# Pos.set_data(1*i, 2*i)
		# return Pos,

	## - First loop
	if (initcond == 0):
		figure = plt.figure()
		# figure, axis = plt.subplots()
		# plt.axis([-axlx, axlx, -axly, axly])
		ax = plt.axes(xlim=(-axlx, axlx), ylim=(-axly, axly))
		plt.grid(1)
		# plt.ion()
		# Pos, = plt.plot(MessageMeasurement.pose2d.x, MessageMeasurement.pose2d.y)
		global Pos
		Pos, = ax.plot([], [], lw = 2)
		# plt.show(figure)
		markerScale = 3
		# anim = animation.FuncAnimation(figure, animatePlot, init_func=initAnimation, frames = 25, interval = 1, blit = True)
		# anim = animation.FuncAnimation(figure, animatePlot, init_func=initAnimation, frames = 200, interval = 20, blit = True)
		# anim = animation.FuncAnimation(figure, animatePlot, init_func=initAnimation, frames = 30, interval = 1, blit = True)
		# Pos, = ax.plot(0)
		# Pos, = plt.plot(0)
		initcond = 1

	## Generate quiver plot
	# global MessageMeasurement
	# plt.quiver(MessageMeasurement.x, MessageMeasurement.y, MessageMeasurement.theta)
	# plt.quiver(MessageEKF.pose2d.x, MessageEKF.pose2d.y, MessageEKF.pose2d.theta, cmap='gray')
	# of = np.rad2deg(MessageMeasurement.theta) - 90
	# of = np.rad2deg(MessageMeasurement.pose2d.theta) - 90
	# Pos = plt.plot(MessageMeasurement.x, MessageMeasurement.y, 'b', marker=(3, 0, of), markersize = markerScale / 3)
	# Pos, = plt.plot(MessageMeasurement.x, MessageMeasurement.y)
	Pos, = plt.plot(MessageMeasurement.pose2d.x, MessageMeasurement.pose2d.y)
	# if (loopcounter > 4):
		# plt.ioff()
	# plt.ion()

	# interactive(True)
	# matplotlib.interactive(False)
	# interactive(False)
	# plt.ioff()
	# plt.show()
	plt.show(block=False)
	# plt.show(block=True)
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
	# installed. The extra_args ensure that the x264 codec is used, so that
	# the video can be embedded in html5.  You may need to adjust this for
	# your system: for more information, see
	# http://matplotlib.sourceforge.net/api/animation_api.html
	# anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
	# anim.save('video', fps=30, extra_args=['-vcodec', 'libx264'])

	# plt.savefig('L%d.png' % loopcounter)
	# plt.show(block=False)
	## Reset subscriber flags
	global flagSubscriber1
	flagSubscriber1 = False
	global flagSubscriber2
	flagSubscriber2 = False

	return 0

class ProcessPlotter(object):
	def __init__(self):
		# self.x = []
		# self.y = []
		global samples
		samples = 2 ## Sufficient to connect markers
		self.x1 = np.zeros(samples)
		self.y1 = np.zeros(samples)
		self.x2 = np.zeros(samples)
		self.y2 = np.zeros(samples)
		self.x3 = np.zeros(samples)
		self.y3 = np.zeros(samples)
		# self.y = range(samples)

	def terminate(self):
		plt.close('all')

	def poll_draw(self):

		def call_back():
			# global samples = 10
			samples
			# ofd = - 90
			ofd = - 135
			# plot(0, 0)
			self.ax.axis([-axlx, axlx, -axly, axly])
			while 1:
				if not self.pipe.poll():
					break

				data = self.pipe.recv()

				if data is None:
					self.terminate()
					return False

				else:
					# self.x.append(data[0])
					# self.y.append(data[1])
					for i in range(samples - 1):
						self.x1[i + 1] = self.x1[i]
						self.y1[i + 1] = self.y1[i]
						self.x2[i + 1] = self.x2[i]
						self.y2[i + 1] = self.y2[i]
						self.x3[i + 1] = self.x3[i]
						self.y3[i + 1] = self.y3[i]
					self.x1[0] = data[1]
					self.y1[0] = data[2]
					self.x2[0] = data[4]
					self.y2[0] = data[5]
					self.x3[0] = data[7]
					self.y3[0] = data[8]
					# of = np.rad2deg(data[3]) - 90
					
					of1 = np.rad2deg(data[3]) + ofd
					of2 = np.rad2deg(data[6]) + ofd
					of3 = np.rad2deg(data[9]) + ofd
					self.ax.plot(self.x1, self.y1, 'r', marker = (3, 0, of1), markersize = 20)
					self.ax.plot(self.x2, self.y2, 'g', marker = (3, 0, of2), markersize = 15)
					self.ax.plot(self.x3, self.y3, 'b', marker = (3, 0, of3), markersize = 12)
					# global loop
					# print('L', loop)
					# print(loopcounter)
					# global loopcounter
					if data[0]: # Clear every so often
						print('Clear')
						#matplotlib.pyplot.cla()
						self.ax.cla()
						# initPlot(self.ax)
						self.ax.grid(1)
						self.ax.axis([-axlx, axlx, -axly, axly])

			self.fig.canvas.draw()
			return True

		return call_back

	def __call__(self, pipe):
		print('starting plotter...')

		self.pipe = pipe
		self.fig, self.ax = plt.subplots()
		plt.grid(1)
		self.gid = gobject.timeout_add(0, self.poll_draw())

		print('...done')
		plt.ioff()
		plt.show()

class NBPlot(object):
	def __init__(self):
		self.plot_pipe, plotter_pipe = Pipe()
		self.plotter = ProcessPlotter()
		self.plot_process = Process(target=self.plotter,
									args=(plotter_pipe,))
		self.plot_process.daemon = True
		self.plot_process.start()

	def plot(self, finished=False):
		send = self.plot_pipe.send
		if finished:
			send(None)
		else:
			if not loopcounter % plotRefreshRate:
				reset = 1
			else:
				reset = 0
			data = [reset, 
						MessageMeasurement.pose2d.x, MessageMeasurement.pose2d.y, MessageMeasurement.pose2d.theta, 
						MessageEKF.odompose2d.x, MessageEKF.odompose2d.y, MessageEKF.odompose2d.theta,
						MessageEKF.ekfpose2d.x, MessageEKF.ekfpose2d.y, MessageEKF.ekfpose2d.theta]
			# print(MessageEKF.ekfpose2d.x, MessageEKF.ekfpose2d.y, MessageEKF.ekfpose2d.theta)
			# print(MessageEKF.odompose2d.x, MessageEKF.odompose2d.y, MessageEKF.odompose2d.theta)
			send(data)
			flagSubscriber1 = False
			flagSubscriber2 = False


## Node main function

def main():
	""" Node main function

		More details
	"""

	##! Initiliaze:
	## - Refer to global variable so that it can be changed
	global loopcounter

	## - Initialize rospy
	# roscpp_initialize(sys.argv)
	rospy.init_node('rollo_visualization', anonymous=True)

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
	# plt.ion()
	# processPlotting = multiprocessing.Process(target = generatePlot(0), args=())
	# processPlotting = Process(target = generatePlot(0), args=())
	# processPlotting.daemon = True
	# processPlotting.start()
	# print('1')
	# print(multiprocessing.current_process())
# multiprocessing.current_process()
	pl = NBPlot()
	# plt.ioff()
	# plt.show()
	# if loopcounter == 20:
		# plt.show()


	while not rospy.is_shutdown():
		## Main loop
		# rospy.loginfo("[Rollo][%s][Main] Loop: %d", NodeName, loopcounter) # //DB
		if not loopcounter % 10:
			rospy.loginfo("[Rollo][%s][Main] Loop: %d", NodeName, loopcounter) # //DB

		if (flagSubscriber1 == True) and (flagSubscriber2 == True):
			# rospy.loginfo("[Rollo][%s][Main] Generate and update plot", NodeName) # //DB
			# generatePlot(1)
			pl.plot()
			# Pos, = plt.plot(MessageMeasurement.x, MessageMeasurement.y)
			# global animatePlot
			# anim = animation.FuncAnimation(figure, animate, frames = 10, interval = 4, blit = True)
			# anim.save('basic_animation.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

		## Sleep to conform node frequency rate
		rosrate.sleep()

		## Update loop counter
		loopcounter = loopcounter + 1
		# processPlotting.join()
		## Main loop end

	# rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
