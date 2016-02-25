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

## Import ROS libraries
import roslib
import sys
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
 * Subscribe to preprocessor topic
 * Subscribe to EKF topic
 * TODO later
 * Implement saving generated images to a path
 * Implement a duration parameter
 * Add references from node to topics and console colours
'''

## Global variables

## Node name using console codes
NodeName = "VIS "

## Visualize rate [Hz] == [fps]
# rate = 25 # 25 [Hz] = 25 [fps]
rate = 1 # 25 [Hz] = 25 [fps]

## Message for measurement data
# MessageMeasurement = 0

## Message for EKF data
# MessageEKF = 0

## Loop counter
loopcounter = 0

## Global flags

# if (loopcounter == 0):
## Global flag 1
flagSubscriber1 = False

## Global flag 2
flagSubscriber2 = False

## Set pyplot to be non-interactive
plt.ioff()

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

## Generate and update plot

def generatePlot():
	rospy.loginfo("[Rollo][%s][generatePlot] Init", NodeName) # //DB

	## Generate quiver plot
	plt.quiver(MessageMeasurement.x, MessageMeasurement.y, MessageMeasurement.theta)
	plt.quiver(MessageEKF.pose2d.x, MessageEKF.pose2d.y, MessageEKF.pose2d.theta, cmap='gray')
	if (not loopcounter % 100):
		plt.hold(False)
	plt.draw()

	## Reset subscriber flags
	global flagSubscriber1
	flagSubscriber1 = False
	global flagSubscriber2
	flagSubscriber2 = False

	return 0

## Node main function

def main():
	## Initiliaze
	### Initialize rospy
	# roscpp_initialize(sys.argv)
	rospy.init_node('rollo_visualization', anonymous=True)

	plt.axis([-4, 4, -4, 4])
	plt.ion()
	plt.show()

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
		rospy.loginfo("[Rollo][%s][Main] Generate and update plot", NodeName) # //DB

		if (flagSubscriber1 == True) and (flagSubscriber2 == True):
			rospy.loginfo("[Rollo][%s][Main] Generate and update plot", NodeName) # //DB
			generatePlot()

		## Sleep to conform node frequency rate
		rosrate.sleep()

		loopcounter =+ 1
		## Main loop end

	# rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
