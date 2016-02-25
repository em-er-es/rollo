/**
 * @file rollo_control.cpp
 * @author Rabbia Asghar
 * @author Ernest Skrzypczyk
 * 
 * @date 18/2/16
 * 
 * @brief Convert input from keyboard and publish control commands for Rollo
 * 
 * Command prototype: <b>rosrun rollo rollo_control _rate:=10</b>
 * \param rate Running frequency of the node <!10 [Hz]>
 *
 * Robot control using following key sets
 * 
 * - - - - - - - - - - - - - - - - - - -
 * 
 * <pre>
 *    q    w    e           u    i    o
 *    a    s    d    f/F    j    k    l
 *    z    x    c           m    ,    .
 * </pre>
 *
 * - - - - - - - - - - - - - - - - - - -
 * Left key set:
 * - q/e : increase/decrease speeds 0.1 and -0.1
 * - w/s : increase/decrease only linear speed by 0.1
 * - a/d : increase/decrease only angular speed by 0.1
 * - z/c : increase/decrease speeds 0.1 and -0.1
 * - x : reset angular speed
 *
 * - - - - - - - - - - - - - - - - - - -
 * 
 * Independent key set:
 * - f/F : full speed forwards/backwards
 *
 * - - - - - - - - - - - - - - - - - - -
 * 
 * Right key set:
 * - u/o : increase/decrease set speeds for diagonal movement forwards
 * - i/, : increase/decrease set speeds for forward/backward movement
 * - j/l : increase/decrease set speeds for rotations
 * - m/. : increase/decrease set speeds for diagonal movement backwards
 * - k : stop
 *
 * - - - - - - - - - - - - - - - - - - -
 * 
 * Global key set:
 * - * : stop
 * - <CTRL>-C : quit
 * 
 * Python script available online used as reference.
 * @see https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
 */

/* TODO
 * FIX DOXYGEN documentation format
 * Double check corresponding key sets
 *! Problem with delay: keys can be hold and the command registers and sends old strokes
 * TODO later
 * Values are not kept at their previous level, a linear decline might be better
// */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>
#include <iostream>
#include "rollo.hpp"


/**
 * @brief Global variables.
 * 
 */

//! Node name using console codes
char NodeName[20] = C2 CT CR; // The size is necessary for the GNU/Linux console codes //COLOR
// char NodeName[20] = CT; // The size is necessary for the GNU/Linux console codes //COLOR

// Topics
//! Topic for commands generated expressed in linear and angular velocity
char TopicCmdVel[64] = TOPIC_CTRL_CMD_VEL;

//! Limit velocity forward
double LimitVelocityF = 1;
//! Limit velocity reverse
double LimitVelocityR = -1;

//! Left key set velocity step
double LKeysSteps = 0.1;
//! Right key set linear velocity step
double RKeysLinearV = 0.4;
//! Right key set angular velocity step
double RKeysAngularV = 1;


/**
 * @brief Keyboard keystroke
 *
 * Check if a key is pressed on keyboard and return it.
 * 
 * \param NONE
 * 
 * @return 1 if a key is pressed on keyboard, otherwise 0.
 * @see https://github.com/sdipendra/ros-projects/blob/master/src/keyboard_non_blocking_input/src/keyboard_non_blocking_input_node.cpp
 */

//TODO doxygen documentation on what is being done
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF) {
		ungetc(ch, stdin);

		return 1;
		}
// Command: rosrun rollo rollo_control _rate:=1
	return 0;
}


/**
 * @brief Decode key
 *
 * Compute linear and angular command velocities based on keyboard input.
 * Key pressed character @p <key> as input argument.
 * 
 * \param character Character to be decoded
 * \param &Speed Linear velocity
 * \param &Turn Angular velocity
 * 
 * @return NULL
 * @see https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
 */

void decodeKey (char character, double &Speed, double &Turn)
{
	switch (character){
		//! Left key set control
		case 'q':	Speed += LKeysSteps; Turn -= LKeysSteps; break;
		case 'w':	Speed += LKeysSteps; Turn = Turn; break;
		case 'e':	Speed += LKeysSteps; Turn += LKeysSteps; break;
		case 'a':	Speed = Speed; Turn += LKeysSteps; break;
		case 's':	Speed -= LKeysSteps; Turn = Turn; break;
		case 'd':	Speed = Speed; Turn -= LKeysSteps; break;
		case 'z':	Speed -= LKeysSteps; Turn += LKeysSteps; break;
		case 'x':	Speed = Speed; Turn = 0; break;
		case 'c':	Speed -= LKeysSteps; Turn -= LKeysSteps; break;

		//! Full speed forward/backward
		case 'f':	Speed = 1.0; Turn = 0; break;
		case 'F':	Speed = -1.0; Turn = 0; break;

		//! Right key set control
		//TODO double check with left key set, then get rid of
		// case 'u':	Speed = 1 * RKeysLinearV; Turn = -0.3 * RKeysAngularV; break;
		// case 'i':	Speed = 1 * RKeysLinearV; Turn = 0 * RKeysAngularV; break;
		// case 'o':	Speed = 1 * RKeysLinearV; Turn = 0.3 * RKeysAngularV; break;
		// case 'j':	Speed = 0 * RKeysLinearV; Turn = -1 * RKeysAngularV; break;
		// case 'k':	Speed = 0 * RKeysLinearV; Turn = 0 * RKeysAngularV; break;
		// case 'l':	Speed = 0 * RKeysLinearV; Turn = 1 * RKeysAngularV; break;
		// case ',':	Speed = -1 * RKeysLinearV; Turn = 0 * RKeysAngularV; break;
		// case '.':	Speed = -1 * RKeysLinearV; Turn = 0.3 * RKeysAngularV; break;
		// case 'm':	Speed = -1 * RKeysLinearV; Turn = -0.3 * RKeysAngularV; break;
		case 'u':	Speed = 1 * RKeysLinearV; Turn = 0.3 * RKeysAngularV; break;
		case 'i':	Speed = 1 * RKeysLinearV; Turn = 0 * RKeysAngularV; break;
		case 'o':	Speed = 1 * RKeysLinearV; Turn = -0.3 * RKeysAngularV; break;
		case 'j':	Speed = 0 * RKeysLinearV; Turn = 1 * RKeysAngularV; break;
		case 'k':	Speed = 0 * RKeysLinearV; Turn = 0 * RKeysAngularV; break;
		case 'l':	Speed = 0 * RKeysLinearV; Turn = -1 * RKeysAngularV; break;
		case 'm':	Speed = -1 * RKeysLinearV; Turn = 0.3 * RKeysAngularV; break;
		case ',':	Speed = -1 * RKeysLinearV; Turn = 0 * RKeysAngularV; break;
		case '.':	Speed = -1 * RKeysLinearV; Turn = -0.3 * RKeysAngularV; break;

		//! Default value
		default:	Speed = 0; Turn = 0; break;
	}

//! Velocity limits

	//! Linear velocity limits
	if (Speed > LimitVelocityF)
			Speed = LimitVelocityF;
	if (Speed < LimitVelocityR)
			Speed = LimitVelocityR;

	//! Angular velocity limits
	//TODO Use global variables for these values
	// if (Turn > LimitVelocityF)
	if (Turn > 0.55)
			Turn = LimitVelocityF;
	// if (Turn < LimitVelocityR)
	if (Turn < -0.60)
			Turn = LimitVelocityR;

	//! Print decoded velocities
	ROS_INFO("[Rollo][%s][DecodeKey] Character [%c] => Speed [%f] Turn [%f]", NodeName, character, Speed, Turn);
}


/**
 * @brief Node main
 *
 * Initialize variables and nodehandle, read and translate input information into command messages.\n
 * 
 * \param rate Running frequency of the node <!10 [Hz]>
 * 
 * Publish to command velocity topic as specified in configuration header file according to format @p geometry_msgs::Twist
 * 
 * @return 0
 */
//* Publish to command velocity topic as specified in configuration header @file rollo.hpp according to format @p geometry_msgs::Twist

int main(int argc, char **argv)
{
//! # Algorithm structure
//! ## Initialization
ros::init(argc, argv, "rollo_control");
ros::start();

//! - Initialize nodehandle for publisher
ros::NodeHandle RolloControlNode;

//! - Publisher initialization with topic, message format and queue size definition
ros::Publisher RolloTwist = RolloControlNode.advertise<geometry_msgs::Twist>(TopicCmdVel, 1024);

//! - Node arguments using command line
int rate_frequency;

//! - Initialize node parameters from launch file or command line.
//! Use a private node handle so that multiple instances of the node can be run simultaneously
//! while using different parameters.
ros::NodeHandle private_node_handle_("~");
private_node_handle_.param("rate", rate_frequency, int(100));

//! - Publishing rate [Hz]
ros::Rate frequency(rate_frequency); 

//! - Publisher variables for conventional messages
geometry_msgs::Twist PubRolloCmd;

//! - Initialize variables for computing linear and angular velocity of the robot
double Speed = 0;
double Turn = 0;

//! - Initialize character holder
char c = 0;

//! ## Main loop
while (ros::ok()) {

	//! - Check if a key is pressed
	if (kbhit()) {
		//! - Read character
		c  =  getchar();

		//! - Decode key pressed
		decodeKey(c, Speed, Turn);
	}

	//! - Prepare message to publish linear and angular velocities
	PubRolloCmd.linear.x = Speed;
	PubRolloCmd.angular.z = Turn;

	//! - Print message with velocities
	ROS_INFO("[Rollo][%s][Pub] Linear speed [%f] Angular speed [%f]", NodeName, PubRolloCmd.linear.x, PubRolloCmd.angular.z);
	//! - Publish message in Twist format
	RolloTwist.publish(PubRolloCmd);

	//! - ROS spinOnce
	ros::spinOnce();

	//! - Sleep to connform node frequency rate
	frequency.sleep();

}
//! ## Main loop end

return 0;
}
