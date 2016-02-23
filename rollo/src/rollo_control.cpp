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
 * q/z : increase/decrease max speeds by 10%
 * w/x : increase/decrease only linear speed by 10%
 * e/c : increase/decrease only angular speed by 10%
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
#include <termios.h>
#include <sstream>
#include <iostream>
#include "rollo.hpp"


/**
 * @brief Global variables.
 * 
 */
 
char NodeName[20] = C2 CT CR; // The size is necessary for the GNU/Linux console codes //COLOR
// char NodeName[20] = CT; // The size is necessary for the GNU/Linux console codes //COLOR

char TopicWheelSpeed[64] = TOPIC_COMM_WS;
char TopicCmdVel[64] = TOPIC_CTRL_CMD_VEL;


/**
 * @brief getCharacter function
 *
 * Reads pressed key on keyboard and returns it.
 * @return Char key pressed on keyboard
 * @see https://github.com/sdipendra/ros-projects/blob/master/src/keyboard_non_blocking_input/src/keyboard_non_blocking_input_node.cpp
 */
 
char getCharacter()
{
	fd_set set;
	struct timeval timeout;
	char buffer = 0;
	int rv;
	int length = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);

	timeout.tv_sec = 0;
	timeout.tv_usec = 1000; // Too small period, only 1ms? //~Q
	// Shouldn't this be directly or indirectly related to rate? //~Q

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios settings = {0};
	if (tcgetattr(filedesc, &settings) < 0)
		ROS_ERROR("tcsetattr()");
	settings.c_lflag &= ~ICANON;
	settings.c_lflag &= ~ECHO;
	settings.c_cc[VMIN] = 1;
	settings.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &settings) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if (rv == -1)
		ROS_ERROR("select()");
	else if (rv == 0)
		// ROS_INFO("[Rollo][%s][getCharacter] Key pressed: NONE", NodeName);
		ROS_INFO("[Rollo][%s][getCharacter] No key pressed", NodeName);
	else {
		read(filedesc, &buffer, length);
		ROS_INFO("[Rollo][%s][getCharacter] Key pressed: %c", NodeName, buffer);
	}

	settings.c_lflag |= ICANON;
	settings.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &settings) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buffer);
}

/**
 * @brief Decode key
 *
 * Computes command linear and angular velocities depending on the keyboard key pressed.
 * The function takes keyboard key pressed character @p as input argument. 
 * Parameters declared by reference: @p &x, @p &th, @p &speed, and @p &turn.
 * @return NULL
 * @see https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
 */
void decodeKey (char character, double &x, double &th, double &speed, double &turn)
{
/*
moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
        }

speedBindings = {
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }
// */

	switch (character){
		case 'i':		x = 1;	th = 0;		break;
		case 'o':		x = 1;	th = -1;	break;
		case 'j':		x = 0;	th = 1;		break;
		case 'l':		x = 0;	th = -1;	break;
		case 'u':		x = 1;	th = 1;		break;
		case ',':		x = -1;	th = 0;		break;
		case '.':		x = -1;	th = 1;		break;
		case 'm':		x = -1;	th = -1;	break;
		case 'q':		speed *= 1.1;	turn *= 1.1;		break;
		case 'z':		speed *= 0.9;	turn *= 0.9;		break;
		case 'w':		speed *= 1.1;	turn *= 1;		break;
		case 'x':		speed *= 0.9;	turn *= 1;		break;
		case 'e':		speed *= 1;		turn *= 1.1;		break;
		case 'c':		speed *= 1;		turn *= 0.9;		break;
		default:		x = 0;	th = 0;	speed = 0.5; turn = 1;	break;
	}

	ROS_INFO("[Rollo][%s][DecodeKey] Character [%c] x [%f] th [%f] speed [%f] turn [%f]", NodeName, character, x, th, speed, turn);
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
private_node_handle_.param("rate", rate_frequency, int(10));

//! Publishing rate [Hz]
ros::Rate frequency(rate_frequency); 

//! Publisher variables for conventional messages
geometry_msgs::Twist PubRolloTwist; // Or PubRolloCmd? //Q

//! Initialize variables for computing linear and angular velocity of the robot
double x = 0;
double th = 0;
double speed = 0.5;
double turn = 1;


//! Main loop
while (ros::ok()) {
	//! Read if any keyboard key is pressed
	int c = 0; // Initialize each loop? //Q
	c = getCharacter();

	//! Decode key pressed
	if (c != 0) 
		decodeKey(c, x, th, speed, turn);


	//! Compute linear and angular velocity
	PubRolloTwist.linear.x = x * speed; 
	PubRolloTwist.angular.z = th * turn;


	//! Publish message in Twist format
	ROS_INFO("[Rollo][%s][Pub] Linear speed [%f] angular speed [%f]", NodeName, PubRolloTwist.linear.x, PubRolloTwist.angular.z);
	RolloTwist.publish(PubRolloTwist);

	//! ROS spinOnce
	ros::spinOnce();

	//! Sleep before running loop again 
	frequency.sleep();

}
//! Main loop end

return 0;
}
