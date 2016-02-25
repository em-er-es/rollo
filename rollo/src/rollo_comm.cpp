/**
 * @file rollo_comm.cpp
 * @author Rabbia Asghar, Ernest Skrzypczyk
 * @date 18/2/16
 * @brief Communication between ROS and Rollo
 *
 * Provides basic communication structure between ROS holding nodes used for localization and Rollo.
 * Main aspects include:
 *  - decoding linear and angular velocities provided by control node
 *  - translate and send message to Rollo
 *  - publish decoded velocities
 *  - square test or n-th order
 *  - emergency procedure
 *
 * @see https://github.com/em-er-es/rollo/
 */


#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "string.h"
#include "rollo.hpp"
#include "geometry_msgs/Twist.h"
#include "rollo/WheelSpeed.h"
#include "udp.h"


// Naming conventions:
// Global and important variables: CapitalLettersFullName
// Local, temporary and irrelevant variables: shortlowercase
// Functions priority: CapitalLettersFunctions
// Functions conventional: firstLowerLetterFunction
// Functions and variables special: _FullDescription
// Comment switches: Debug //DB
// Comment switches: Verbose //VB
// Comment switches: Color //COLOR
// Comment switches: Any other switch //RelevantCapitalLettersAbbreviation
// Comment tags: Tasks needed to be fixed //FIX
// Comment tags: Tasks left to do //TODO
// Comment tags: Question //Q
// Comment tags: Suggestion //S
// Comment tags: Correct comment //CRC
// Comment tags: Correct variable //CRV


/* TODO
 *! Define all possible messages by contructing them out of three bytes
 *! * 1: Movement/operation byte, 2: Left wheel speed, 3: Right wheel speed
 *! Run the node either at a given frequency or if there are problems with too much data for Rollo, at a rate of 60 or 30 Hz
 *!/ Make the node accepts parameters for -ip address-, udp port and node frequency
 *! Add publisher for custom message oncluding timestamped velocities [rpm] of Rollo for EKF node
 * Get rid of warnings for multi-character constants and overflow
 * TODO LATER
 *! Priorotize safety over control
 * Define a function that will receive UDP packets
 * Define function to parse the ip address properly
// */


/**
 * @brief Global variables updated in the @p SubscriberCallback function, processed and used to send commands to the specified @p IP adress at the @p UDP port.
 *
 */

//! Node name using console codes
char NodeName[20] = C3 CM CR; // The size is necessary for the GNU/Linux console codes //COLOR
// char NodeName[20] = CM; // The size is necessary for the GNU/Linux console codes //COLOR

// Topics
//! Topic for wheel speed containing the actual speed of wheel, preferably extracted from encoders or if not available by using a lookup table
char TopicWheelSpeed[64] = TOPIC_COMM_WS;
//! Topic for commands generated by control node expressed in linear and angular velocity
char TopicCmdVel[64] = TOPIC_CTRL_CMD_VEL;

//! Rollo default IP: 192.168.0.120
// Local IP address range: 130.251.13.90-99

// There is no reason for these variables to be signed
char ip[16] = "192.168.0.120"; //!< Ip address statically assigned
// unsigned int port = 900; //! UDP port
int port = 900; //!< UDP port

// Alternatively the global variables were used
// double x, z, tol = 0.01; //!< Linear and angular velocities from the control node
double tol = 0.01; //!< Tolerance for determining linear and angular velocities from the control node
int v_l, v_r; //!< Velocities for both wheels

unsigned const int nb = 3; //!< Number of bytes in the message
char Message[nb] = {0x7b, 0x50, 0x10}; //!< Message combined, complete stop default

//! Emergency variables
char MessageEmergencyStop[nb] = {0x7b, 0x50, 0x10}; //!< Emergency message - complete stop
double lastMessageTime = 0; //!< Last message from control node
double currentTime = 0; //!< Current time holder
int EmergencyTime = 10; //!< Emergency time [s]
bool FlagEmergency = 0; //!< Emergency flag

char Mode[2]; //!< Message mode description
int VelocityL, VelocityR; //!< Message velocities description

unsigned int loopcounter = 1; //!< Loop counter for debugging purpose

double RolloMax = ROLLO_SPEED_MAX; //!< Maximum speed of the Rollo
double RolloMin = ROLLO_SPEED_MIN; //!< Minimum speed of the Rollo
double RolloRange = RolloMax - RolloMin; //! Range of speed of the Rollo


/**
 * @brief Decode linear and angular velocities
 *
 * Velocities are decoded and stored as partial bytes of the UDP packet
 * Parameters declared: @p x, @p z, @p &Message, @p VelocityL, @p VelocityR.
 * @return 0
 */

int decodeVelocities(double x, double z, char *Message, int &VelocityL, int &VelocityR) //S Could read the velocities and transform them later on
{
//! Determine corresponding operation mode based on velocities

/* Possible values for the wheel velocities parsed from cruderollo.sh script
		INFO="6%";
		INFO="12%";
		INFO="19%";
		INFO="25%";
		INFO="31%";
		INFO="38%";
		INFO="44%";
		INFO="50%";
		INFO="56%";
#		INFO="100%"; // Not used
// */

	// if (! (loopcounter % 10)) ROS_INFO("[Rollo][%s][DecodeVelocities] ([%f], [%f] tol: %f)", NodeName, x, z, tol); //DB

	// if (x == 0) { //! Ideal case
	//! Since control node can provide abstract control values, an ideal case could be used for decoding velocities
	//! This is discouraged, since using alternative control methods would probably have a realistic value set
	if (fabs(x) < tol) { //! Linear velocity is approx 0

		if (z == 0) Message[0] = 0x7b; //! Complete stop
		// if (abs(x) < tol) Message[0] = 0x7b; //! Complete stop
		// else if (z > 0) Message[0] = 0x7f; //! Right rotation
		// else if (z < 0) Message[0] = 0x7e; //! Left rotation
		// else if (z < 0) Message[0] = 0x7f; //! Right rotation
		// else if (z > 0) Message[0] = 0x7e; //! Left rotation
		else if (z < tol) Message[0] = 0x7f; //! Right rotation
		else if (z > tol) Message[0] = 0x7e; //! Left rotation

	Message[1] = 0x50; Message[2] = 0x10; //! Lowest speeds for previous modes

	// } else if (x != 0) { //! Determine speeds
	} else if (fabs(x) > tol) { //! Linear velocity is above tolerance threshold
		//! Determine speeds based on the position of the "dial" z\n
		//! |-a-|- - -*-b- - - - -|\n
		//! -1   z   0       1

		double tx = x * RolloRange; //! Temporary velocity holder
		double a = (z + 1.001); //! Eliminate problems with dividing through zero by adding a small number to variables
		double b = 2 - (z + 1.001);
		//! Calculate velocities according to relation expressed in linear and angular velocities ratio
		v_l = (b / a * tx) / RolloMin;
		v_r = (a / b * tx) / RolloMin;

		//! Translate velocities for Rollo
		//! Left velocity - Second byte
		switch (abs(v_l)){
			case 0: Message[1] = 0x50; VelocityL = 6; break;
			case 1: Message[1] = 0x55; VelocityL = 12; break;
			case 2: Message[1] = 0x56; VelocityL = 19; break;
			case 3: Message[1] = 0x57; VelocityL = 25; break;
			case 4: Message[1] = 0x59; VelocityL = 31; break;
			case 5: Message[1] = 0x5F; VelocityL = 38; break;
			case 6: Message[1] = 0x60; VelocityL = 44; break;
			case 7: Message[1] = 0x61; VelocityL = 50; break;
			case 8: Message[1] = 0x62; VelocityL = 56; break;
			case 9: Message[1] = 0x62; VelocityL = 56; break;
			default: Message[1] = 0x50; VelocityL = 6; break;
		}

		//! Right velocity - Third byte
		switch (abs(v_r)){
			case 0: Message[2] = 0x10; VelocityR = 6; break;
			case 1: Message[2] = 0x11; VelocityR = 12; break;
			case 2: Message[2] = 0x25; VelocityR = 19; break; //! Temporary fix for errartic behaviour of Rollo
			case 3: Message[2] = 0x13; VelocityR = 25; break;
			case 4: Message[2] = 0x1A; VelocityR = 31; break;
			case 5: Message[2] = 0x1B; VelocityR = 38; break;
			case 6: Message[2] = 0x1C; VelocityR = 44; break;
			case 7: Message[2] = 0x1D; VelocityR = 50; break;
			case 8: Message[2] = 0x24; VelocityR = 56; break;
			case 9: Message[2] = 0x24; VelocityR = 56; break;
			default: Message[2] = 0x10; VelocityR = 6; break;
		}

		//! Determine forward or backward movement based on linear velocity
		// if (x > 0) {
		if (x > tol) {
			Message[0] = 0x7c; 
			// Mode[0] = 'F';
			// *(Mode) = 'F';
		}
		// else if (x < 0) {
		else if (-x < tol) {
			Message[0] = 0x7d;
			// *(Mode) = 'B';
		}
		// } else if (z > 0) *(Mode) = 'R';
		// else if (z < 0) *(Mode) = 'L';
		// } else if (z > tol) *(Mode) = 'R';
		// else if (z < tol) *(Mode) = 'L';

	}

/* //DB
	if (! (loopcounter % 10)) {
		// ROS_INFO("[Rollo][%s][DecodeVelocities] ([%f], [%f]) => [[%c][%c][%c]]", NodeName, x, z, Message[0], Message[1], Message[2]); //DB
		// ROS_INFO("[Rollo][%s][DecodeVelocities] M: %c ([%f], [%f]) => [[%d][%d][%d]]", NodeName, Mode, float(x), float(z), Message[0], Message[1], Message[2]); //DB
		ROS_INFO("[Rollo][%s][DecodeVelocities] ([%f], [%f]) => [[%d][%d][%d]]", NodeName, float(x), float(z), Message[0], Message[1], Message[2]); //DB
		// std::cout << "[Rollo][" << NodeName << "][DecodeVelocities] ([" << x << "], [" << z << "])\n" << std::endl; //DB
	}
// */

}

/**
 * @brief Subscriber callback
 *
 * Reads newest velocities from control node and translates them into UDP message
 * Updates latest message time
 *
 * @return 0
 */

void subscriberCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	// double xt = msg->linear.x;
	// double zt = msg->angular.z;
	// decodeVelocities(xt, zt, Message); //! Update the UDP message
	// decodeVelocities(msg->linear.x, msg->angular.z, Message); //! Update the UDP message
	// decodeVelocities(Message); //! Update the UDP message
	decodeVelocities(msg->linear.x, msg->angular.z, Message, VelocityL, VelocityR); //! Update the UDP message
	lastMessageTime = ros::Time::now().toSec(); //! Update last message time
	FlagEmergency = 0;
}

/**
 * @brief Send UDP packets
 *
 * Send provided message using included UDP library command @p udp_client_server::udp_client.send()
 *
 * Parameters declared by reference: @p &ip, @p &port, @p &message.
 *
 * @return Bytes sent
 */

int udpSend(char ip[16], int port, char *Message)
{
	udp_client_server::udp_client udpClient(ip, port); //! Client initialization
	int bs = udpClient.send(Message, nb); //! Send UDP message
	if (bs == nb) { //! Check if number of bytes sent is equal to bytes of composed message
		// ROS_INFO("[Rollo][%s][UDP] Message: [%d|%d|%d]", NodeName, Message[0], Message[1], Message[2]); //DB
		// ROS_INFO("[Rollo][%s][UDP] UDP packets sent successfully", NodeName); //VB
		// ROS_INFO("[Rollo][%s][UDP][Mode, v_l, v_r]: %s, %f, %f", NodeName, *(Mode), v_l, v_r); //DB
		ROS_INFO("[Rollo][%s][UDP][Mode, v_l, v_r]: %s, %f, %f", NodeName, Mode, VelocityL, VelocityR); //VB
	} else if (bs < 0) { //! Error handling
		ROS_INFO("[Rollo][%s][ERR] UDP packets transmission failed", NodeName); //VB
		ROS_ERROR("sendto()");
	}

	return (bs); //! Return bytes sent or error code
}

/**
 * @brief Main function
 *
 * Depending on specified parameters processes data from control node and Rollo
 * and transmits them to appropriate targets or runs a square test of n-th order
 *
 * @return 0
 */

int main(int argc, char **argv)
{
//! Initialize node
ros::init(argc, argv, "rollo_comm"); // Communication node name
ros::start(); // ROS node initialization

//! Initialize nodehandle
ros::NodeHandle RolloCommunicationNode;

//! Initialie subscriber and define topic and message queue size
ros::Subscriber SubRollo = RolloCommunicationNode.subscribe(TopicCmdVel, 1024, subscriberCallback);
ros::Publisher PubRollo = RolloCommunicationNode.advertise<rollo::WheelSpeed>(TopicWheelSpeed, 1024); //! Publish velocities as [rpm]

//! Initialize node arguments using command line
int rate_frequency = 0.1;

// Command: rosrun rollo rollo_communication _rate:=1
//! Initialize node parameters from launch file or command line.
//! Use a private node handle so that multiple instances of the node
//! can be run simultaneously while using different parameters.
ros::NodeHandle private_node_handle_("~");

//! Node main parameters
private_node_handle_.param("rate", rate_frequency, int(25));
private_node_handle_.param("port", port, int(900));
// IP address parsing seems to be very problematic, therefore a hardcoded assignment has been made for now
// private_node_handle_.param("ip", ip, char(16));
// private_node_handle_.param("ip", ip, "192.168.0.120");
// private_node_handle_.param("ip", ip, const char("192.168.0.120"));
// private_node_handle_.param("ip", ip, const char("192.168.0.120"));
private_node_handle_.param("et", EmergencyTime, int(10));

//! Square test parameters
//! Default values
int square = 0;
double turntime = 6;
double forwardtime = 25;
// The code below also specifies default values, however for debugging it can be disabled with comments more easily
private_node_handle_.param("square", square, int(0));
private_node_handle_.param("forwardtime", forwardtime, double(25));
private_node_handle_.param("turntime", turntime, double(6));

//! Node frequency rate [Hz]
if (rate_frequency < 0) rate_frequency = 1 / (rate_frequency);
ros::Rate frequency(rate_frequency);

//! Initialize subscriber message type
geometry_msgs::Twist SubRolloTwist;
lastMessageTime = ros::Time::now().toSec();

//! Initialize publisher message type
rollo::WheelSpeed PubRolloWheelSpeed;

//! Initialize variables for computing linear and angular velocity of the robot
int rv = 0;

//! Client initialization
udp_client_server::udp_client udpClient(ip, port);


//! Square test
//! - Alternatively this square test could be in control node, however communication node is "closer" to Rollo
	while ((square > 0) && ros::ok()) {

		//! Print information on current run
		ROS_INFO("[Rollo][%s][Square test][%d]", NodeName, square); //DB

		//FIX Get rid of warnings:
		//warning: multi-character character constant [-Wmultichar]
		//warning: overflow in implicit constant conversion [-Woverflow]
		char CmdTurn[4] = {'0x7b', '0x55', '0x11'}; //! Compose turn command
		if (square % 1) CmdTurn[0] = '0x7e'; else CmdTurn[0] = '0x7f'; //! Check square run variable and determine turning direction
		//! For multiple runs the robot would go back and forth providing more reliable data on the actual error
		//! In ideal case even a high order square run would result in the robot being at the initial position with initial orientation
		char CmdForward[3] = {'0x7c', '0x55', '0x11'}; //! Compose forward command
		ROS_INFO("[Rollo][%s][Square test][%d] Turning: %d {dec}", NodeName, square, CmdTurn[0]); //DB
		//! Set initial time
		double initialtime = ros::Time::now().toSec();
		//! Bytes sent, useful for debugging
		int bs;

		// ROS_INFO("[Rollo][%s][Square test] Starting time %g, forward time %g, turning time %g", NodeName, initialtime, forwardtime, turntime); //DB
		ROS_INFO("[Rollo][%s][Square test][%d] Forward time %g, turning time %g", NodeName, square, forwardtime, turntime); //DB

		for (int i = 0; i < 4; i++) { //! Main square loop

			//! Moving forward
			ROS_INFO("[Rollo][%s][Square test][%d] Moving forward: %g [s]", NodeName, square, forwardtime); //DB
			for (int j = 0; j < 3; j++) bs += udpClient.send(CmdForward, nb); //! Send command 3 times
			ros::Duration(forwardtime, 0).sleep(); //! Wait for the specified time to move forward
			// Alternatively use a while loop here and update the command either at the rate of the loop or other

			//! Turning
			ROS_INFO("[Rollo][%s][Square test][%d] Turning: %g [s]", NodeName, square, turntime); //DB
			for (int j = 0; j < 3; j++) bs += udpClient.send(CmdTurn, nb); //! Send command 3 times
			ros::Duration(turntime, 0).sleep(); //! Wait for the specified time to turn
			// Alternatively use a while loop here and update the command either at the rate of the loop or other

		} //! Main square loop end

		double finishtime = ros::Time::now().toSec(); //! Update run finish time
		//! Print duration time
		ROS_INFO("[Rollo][%s][Square test][%d] Finished: %g [s]", NodeName, square, abs(finishtime - initialtime)); //DB

		if (square > 1) square--; else return 0; //! Update square run counter and check for exit condition
		// Alternative approaches for variable check and exit
		// if (! square % 2) square--; else return 0;
		// square--;
		// if (!square) return 0;
	}


//! Main loop
while (ros::ok()) {

	//! Send control command to Rollo
	rv = udpSend(ip, port, Message);

	//! Check if emergency condition has been met
	currentTime = ros::Time::now().toSec();
	if (abs(lastMessageTime - currentTime) > abs(EmergencyTime)) {
		//! Print emergency message
		ROS_INFO("[Rollo][%s][Emergency stop][Emergency time: %d] Apparent loss of connection to control node!", NodeName, EmergencyTime); //DB
		//! Conduct emergency stop
		for (int i = 0; i < 10; i++) //! Send emergency message 10 times
			rv = udpSend(ip, port, MessageEmergencyStop);
		//! Hard condition emergency procedure
		//! Exit with an error code if hard condition is set by using negative values for emergency time
		if (EmergencyTime < 0) return 128;
		//! Soft condition emergency procedure
		FlagEmergency = 1; //! Set emergency flag
		while (FlagEmergency && ros::ok()) {

			//! ROS spinOnce
			ros::spinOnce();

			//! Sleep before running loop again
			frequency.sleep();

		}
	}

/* //DB
	// rv = udpSend("192.168.0.120", port, Message);
	if (rv) {
		// ROS_INFO("[Rollo][%s] udpSend() command executed", NodeName); //DB
		ROS_INFO("[Rollo][%s] udpSend() command exited cleanly", NodeName); //VB
		// usleep(1000000);
		// usleep(10000);
	}
// */

	// //! Publish encoder readings
	// VelocityL = v_l;
	// VelocityR = v_r;
//! Callback function for subscriber

	//! Compose message
	PubRolloWheelSpeed.header.stamp = ros::Time::now();
	// PubRolloWheelSpeed.wheelspeedleft = v_l;
	// PubRolloWheelSpeed.wheelspeedright = v_r;
	PubRolloWheelSpeed.wheelspeedleft = VelocityL;
	PubRolloWheelSpeed.wheelspeedright = VelocityR;

	//! Publish message
	PubRollo.publish(PubRolloWheelSpeed);

	ROS_INFO("[Rollo][%s][Pub] Message: [%d|%d|%d], L[%d], R[%d]", NodeName, Message[0], Message[1], Message[2], VelocityL, VelocityR); //DB
	// ROS_INFO("[Rollo][%s][Pub] TS[%d.%d] L[%d] R[%d]", NodeName, PubRolloWheelSpeed.header.stamp.sec, PubRolloWheelSpeed.header.stamp.nsec, PubRolloWheelSpeed.wheelspeedleft, PubRolloWheelSpeed.wheelspeedright); //DB

	//! ROS spinOnce
	ros::spinOnce();

	//! Sleep before running loop again
	frequency.sleep();

	//! Increase loopcounter
	loopcounter++;
}
//! Main loop end

return 0;

}
