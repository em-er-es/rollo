/**
 * @file rollo_control.cpp
 * @author Rabbia Asghar
 * @date 18/2/16
 * @brief Takes input from keyboard and publishes commands to control Rollo
 *
 * Robot control using following keys
 * ---------------------------
 * Moving around:
 *   u    i    o
 *   j    k    l
 *   m    ,    .
 * ---------------------------
 * q/z : increase/decrease speed by 0.1
 * w/x : increase/decrease only linear speed by 0.1
 * e/c : increase/decrease only angular speed by 0.1
 * ---------------------------
 * Other keys : stop 
 * <CTRL>-C : quit
 * Python script available online used as reference
 * @see https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
 */

// Issues:
// * Problem with delay: keys can be hold and the command registers and sends old strokes
// * Values are not kept at their previous level, a linear decline might be better

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

char NodeName[20] = C2 CT CR; // The size is necessary for the GNU/Linux console codes //COLOR
// char NodeName[20] = CT; // The size is necessary for the GNU/Linux console codes //COLOR
char TopicCmdVel[64] = TOPIC_CTRL_CMD_VEL;

double VelocityFwd_limit = 1;
double VelocityRev_limit = -1;

double LKeysSteps = 0.1;
double RKeysLinearV = 0.4;
double RKeysAngularV = 1;
  

/**
 * @brief kbhit function
 *
 * Checks if a key is pressed on keyboard and returns it.
 * @return 1 if a key is pressed on keyboard, otherwise 0.
 * @see https://github.com/sdipendra/ros-projects/blob/master/src/keyboard_non_blocking_input/src/keyboard_non_blocking_input_node.cpp
 */

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

return 0;
}



/**
 * @brief Decode key
 *
 * Computes command linear and angular velocities depending on the keyboard key pressed.
 * The function takes keyboard key pressed character @p as input argument. 
 * Parameters declared by reference: @p &Speed, and @p &Turn.
 * @return NULL
 * @see https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
 */
void decodeKey (char character, double &Speed, double &Turn)
{
//FIX reverse speed, runs only at 0==6%
	switch (character){
		case 'i':		Speed = 1 * RKeysLinearV; Turn = 0 * RKeysAngularV; break;
		case 'o':		Speed = 1 * RKeysLinearV; Turn = 0.3 * RKeysAngularV; break;
		case 'j':		Speed = 0 * RKeysLinearV; Turn = -1 * RKeysAngularV; break;
		case 'l':		Speed = 0 * RKeysLinearV; Turn = 1 * RKeysAngularV; break;
		case 'u':		Speed = 1 * RKeysLinearV; Turn = -0.3 * RKeysAngularV; break;
		case ',':		Speed = -1 * RKeysLinearV; Turn = 0 * RKeysAngularV; break;
		case '.':		Speed = -1 * RKeysLinearV; Turn = 0.3 * RKeysAngularV; break;
		case 'm':		Speed = -1 * RKeysLinearV; Turn = -0.3 * RKeysAngularV; break;

		case 'q':		Speed += LKeysSteps; Turn -= LKeysSteps; break;
		case 'z':		Speed -= LKeysSteps; Turn += LKeysSteps; break;
		case 'a':		Speed = Speed; Turn += LKeysSteps; break;
		case 'd':		Speed = Speed; Turn -= LKeysSteps; break;
		case 'w':		Speed += LKeysSteps; Turn = Turn; break;
		case 's':		Speed -= LKeysSteps; Turn = Turn; break;
		case 'e':		Speed += LKeysSteps; Turn += LKeysSteps; break;
		case 'c':		Speed -= LKeysSteps; Turn -= LKeysSteps; break;

		case 'f':		Speed = 1.0; Turn = 0; break;
		case 'F':		Speed = -1.0; Turn = 0; break;

		default:		Speed = 0; Turn = 0; break;
	}

//! Turn limits
// if (Turn > 0.55) Turn = 0.55; else if (Turn < -0.60) Turn = -0.6;

//TODO build in restrains for angular velocity, should be -0.6 && 0.5
double VelocityFwd_limit = 1;
double VelocityRev_limit = -1;

	if (Speed > VelocityFwd_limit)
			Speed = VelocityFwd_limit;
	if (Speed < VelocityRev_limit)
			Speed = VelocityRev_limit;
			
	if (Turn > VelocityFwd_limit)
			Turn = VelocityFwd_limit;
	if (Turn < VelocityRev_limit)
			Turn = VelocityRev_limit;

	ROS_INFO("[Rollo][%s][DecodeKey] Character [%c] Speed [%f] Turn [%f]", NodeName, character, Speed, Turn);
}


/**
 * @brief Main
 *
 * Initializes variables, nodehandle, reads and translated input information into command messages.
 * Accepts 1 argument from command line: rate. Default rate is 10 [Hz].
 * The commands are published to topic /Rollo/cmd_vel in configuration in format geometry_msgs::Twist
 * @return 0
 */

int main(int argc, char **argv)
{

//! Initialization 
ros::init(argc, argv, "rollo_control");
ros::start();

//! Initialize nodehandle for publisher
ros::NodeHandle RolloControlNode;

//! Publisher initialization with topic, message format and queue size definition
ros::Publisher RolloTwist = RolloControlNode.advertise<geometry_msgs::Twist>(TopicCmdVel, 1024);

//! Node arguments using command line
int rate_frequency;

// Command: rosrun rollo rollo_control _rate:=1
//! Initialize node parameters from launch file or command line.
//! Use a private node handle so that multiple instances of the node can be run simultaneously
//! while using different parameters.
ros::NodeHandle private_node_handle_("~");
private_node_handle_.param("rate", rate_frequency, int(100));

//! Publishing rate [Hz]
ros::Rate frequency(rate_frequency); 

//! Publisher variables for conventional messages
geometry_msgs::Twist PubRolloTwist; // Or PubRolloCmd? //~Q

//! Initialize variables for computing linear and angular velocity of the robot
double Speed = 0;
double Turn = 0;

char c = 0; 

//! Main loop
while (ros::ok()) {
	
	//! Read if any keyboard key is pressed
	if (kbhit()) {
		//! Read character	
		c  =  getchar();

		//! Decode key pressed
		decodeKey(c, Speed, Turn);
	}
	
	//! Prepare message to publish linear and angular velocity
	PubRolloTwist.linear.x = Speed; 
	PubRolloTwist.angular.z = Turn;


	//! Publish message in Twist format
	ROS_INFO("[Rollo][%s][Pub] Linear speed [%f] Angular speed [%f]", NodeName, PubRolloTwist.linear.x, PubRolloTwist.angular.z);
	RolloTwist.publish(PubRolloTwist);

	//! ROS spinOnce
	ros::spinOnce();

	//! Sleep before running loop again 
	frequency.sleep();

}
//! Main loop end

return 0;
}
