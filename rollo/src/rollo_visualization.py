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
from std_msgs.msg import String

## Import custom messages for Rollo
# from rollo import RolloEKF
# from _rollo_generate_messages_check_deps_EKF.msg import RolloEKF
# from rollo.msg import _EKF # Works
from rollo.msg import EKF # Also works!

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
rate = 25 # 25 [Hz] = 25 [fps]


'''
def subscriberCallbackMeasurement():
	subscriberM = rospy.Subscriber('chatter', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()
'''

def subscriberCallbackEKF(msg):
	rospy.loginfo("[Rollo][%s][Sub] Message: [ %s | %s | %s ]", NodeName, msg.pose2d.x, msg.pose2d.y, msg.pose2d.theta)
	# rospy.loginfo("[Rollo][%s][Sub] Message: [%s]", NodeName, msg.pose2d.x)


## Node main function

def main():
	## Initiliaze
	### Initialize rospy
	# roscpp_initialize(sys.argv)
	rospy.init_node('rollo_visualization', anonymous=True)

	## Set frequency rate for visualization node
	rosrate = rospy.Rate(rate)

	## Subscribe to EKF topic
	# rospy.Subscriber('/Rollo/ekf', std_msgs.msg.Int32, subscriberCallbackMeasurement)
	rospy.Subscriber('/Rollo/ekf', EKF, subscriberCallbackEKF, queue_size = 1024)

	while not rospy.is_shutdown():
		## Main loop

		## Sleep to conform node frequency rate
		rosrate.sleep()

		## Main loop end

	# rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
