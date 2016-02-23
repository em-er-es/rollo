/**
 * @file rollo_comm.cpp
 * @author Rabbia Asghar, Ernest Skrzypczyk
 * @date 18/2/16
 * @brief Communication between nodes and Rollo
 *
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
 * Define a function that will send UDP packets with ip adress, udp port and message as parameters
 * Subscribe to the control node messages for input
 * Define all possible messages by contructing them out of three bytes
 * * 1: Movement/operation byte, 2: Left wheel speed, 3: Right wheel speed
 * Run the node either at a given frequency or if there are problems with too much data for Rollo, at a rate of 60 or 30 Hz
 * Make the node accepts parameters for ip address, udp port and node frequency
 * Add publisher for custom message oncluding timestamped velocities [rpm] of Rollo for EKF node
 * TODO LATER
 * Priorotize safety over control
 * Define a function that will receice UDP packets
 * Define function to parse the ip address properly
// */


/**
 * @brief Global variables updated in the SubscriberCallback function, processed and used to send commands to the specified Ip adress at the UDP port.
 *
 */

char NodeName[20] = C3 CM CR; // The size is necessary for the GNU/Linux console codes //COLOR
// char NodeName[20] = CM; // The size is necessary for the GNU/Linux console codes //COLOR

//! Topics
char TopicWheelSpeed[64] = TOPIC_COMM_WS;
char TopicCmdVel[64] = TOPIC_CTRL_CMD_VEL;

/*
//! Internet protocol version 4 representations
struct IPv4 {
	unsigned char b1, b2, b3, b4;
}

//! Rollo IP: 192.168.0.120
// Local IP address range: 130.251.13.90-99
IPv4 unsigned char b1 = 192;
IPv4 unsigned char b2 = 168;
IPv4 unsigned char b3 = 0;
IPv4 unsigned char b4 = 120;
// */

// unsigned char ip[4] = {b1, b2, b3, b4}; //! Ip address
// unsigned char ip[16]; //! Ip address
// unsigned char ip[16] = "192.168.0.120"; //! Ip address
char ip[16] = "192.168.0.120"; //! Ip address
// unsigned int port = 900; //! UDP port
int port = 900; //! UDP port

double x, z; //! Linear and angular velocities from the control node
int v_l, v_r; //! Velocities for both wheels

// unsigned int nb = 3; //! Number of bytes in the message
unsigned const int nb = 3; //! Number of bytes in the message
char Message[nb] = {0x7b, 0x50, 0x10}; //! Message combined, complete stop default
char MessageEmergencyStop[nb] = {0x7b, 0x50, 0x10}; //! Emergency message - complete stop

char Mode; //! Message mode description
int VelocityL, VelocityR; //! Message velocities description

unsigned int RolloMax = ROLLO_SPEED_MAX; //! Maximum speed of the Rollo
unsigned int RolloMin = ROLLO_SPEED_MIN; //! Minimum speed of the Rollo
unsigned int RolloRange = RolloMax - RolloMin; //! Range of speed of the Rollo


/**
 * @brief Decode linear and angular velocities
 *
 * Velocities are decoded and stored as partial bytes of the UDP packet
 * Parameters declared: @p x, @p z, @p &Message.
 * Parameters declared: @p x, @p z, @p &MessageB1, @p &MessageB2, @p &MessageB3.
 * @return 0
 * @see DOCURL
 */

int decodeVelocities(double x, double z, char *Message) //S Could read the velocities and transform them later on
{
//! Determine corresponding operation mode based on velocities

/* Possible values for the wheel velocities
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

	if (x == 0) {

		if (z == 0) Message[0] = 0x7b; //! Complete stop
		else if (z > 0) Message[0] = 0x7f; //! Right rotation
		else if (z < 0) Message[0] = 0x7e; //! Left rotation

		Message[1] = 0x50; Message[2] = 0x10; //! Lowest speeds for previous modes

	} else if (x != 0) { //! Determine speeds
		//! Based on the position of the "dial" z
		//!    |-a-|---*-b-----|
		//!   -1   z   0       1

		float tx = x * RolloRange;
		float a = (z + 1.001); //! Eliminate problems with diving through zero
		float b = 2 - (z + 1.001);
		v_l = (b / a * tx) / RolloMin;
		v_r = (a / b * tx) / RolloMin;

		switch (v_l){
			case 0:		Message[1] = 0x50; break;
			case 1:		Message[1] = 0x55; break;
			case 2:		Message[1] = 0x56; break;
			case 3:		Message[1] = 0x57; break;
			case 4:		Message[1] = 0x59; break;
			case 5:		Message[1] = 0x5F; break;
			case 6:		Message[1] = 0x60; break;
			case 7:		Message[1] = 0x61; break;
			case 8:		Message[1] = 0x62; break;
			case 9:		Message[1] = 0x62; break;
			default:	Message[1] = 0x50; break;
		}

		switch (v_r){
			case 0:		Message[2] = 0x10; break;
			case 1:		Message[2] = 0x11; break;
			case 2:		Message[2] = 0x25; break; //! Temporary fix for errartic behaviour of Rollo
			case 3:		Message[2] = 0x13; break;
			case 4:		Message[2] = 0x1A; break;
			case 5:		Message[2] = 0x1B; break;
			case 6:		Message[2] = 0x1C; break;
			case 7:		Message[2] = 0x1D; break;
			case 8:		Message[2] = 0x24; break;
			case 9:		Message[2] = 0x24; break;
			default:	Message[2] = 0x10; break;
		}

		if (x > 0) Message[0] = 0x7c; //! Determine forward or backward movement
		else if (x < 0) Message[0] = 0x7d;

	}

	ROS_INFO("[Rollo][%s][DecodeVelocities] ([%f], [%f]) => [[%c][%c][%c]]", NodeName, x, z, Message[0], Message[1], Message[2]); //DB
}

/**
 * @brief Subscriber callback
 *
 * Reads newest velocities from control node and decodes them into UDP message
 * @return 0
 * @see DOCURL
 */

//! Callback function for subscriber
void subscriberCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	x = msg->linear.x;
	z = msg->angular.z;
	decodeVelocities(x, z, Message); //! Update the UDP message
	// Message[2] = MessageB3; //! Update the UDP message
}

/**
 * @brief Send UDP packets
 *
 * Parameters declared by reference: @p &ip, @p &port, @p &message.
 * @return Bytes sent
 * @see DOCURL
 */

// int udpSend(IPv4 ip, unsigned int &port, char udpBuffer[3]) // Could read the velocities and transform them later on
int udpSend(char ip[16], int &port, char udpBuffer[3]) // Could read the velocities and transform them later on
{
	udp_client_server::udp_client udpClient(ip, port); //! Client initialization

	// char udpBuffer[3] = {0x7d, 0x59, 0x31};
	// char udpBuffer[3] = Message;
	int bs = udpClient.send(udpBuffer, nb);
	if (bs == nb) {
		ROS_INFO("[Rollo][%s][UDP] UDP packets sent successfully", NodeName); //DB
		ROS_INFO("[Rollo][%s][UDP][Mode, v_l, v_r]: %f, %f, %f", NodeName, Mode, v_l, v_r);
	} else if (bs < 0) {
		ROS_INFO("[Rollo][%s][ERR] UDP packets transmission failed", NodeName); //DB
		ROS_ERROR("sendto()");
	}

	return (bs);
}

/*
	udpClientlient_server::udpClientlient udpClient(ip, port);
	char udpBuffer[3]={0x7d, 0x59, 0x31};
	udpClient.send(udpBuffer, 3);
	printf("udp_packet_Sent \n");
	sleep(1);
// */


/**
 * @brief Main function
 *
 *
 * @return 0
 */
int main(int argc, char **argv)
{
//! Initialize node
ros::init(argc, argv, "rollo_comm"); // Communication node
ros::start(); // ROS node initialization

//! Initialize nodehandle
ros::NodeHandle RolloCommunicationNode;

//! Initialie subscriber and define topic and message queue size
// ros::Subscriber SubRollo = RolloCommunicationNode.subscribe("/Rollo/control/cmd_vel", 1024, subscriberCallback);
// ros::Publisher PubRollo = RolloCommunicationNode.advertise<rollo::WheelSpeed>("/Rollo/communication/wheelspeed", 1024); //! Publish velocities as [rpm]
ros::Subscriber SubRollo = RolloCommunicationNode.subscribe(TopicCmdVel, 1024, subscriberCallback);
ros::Publisher PubRollo = RolloCommunicationNode.advertise<rollo::WheelSpeed>(TopicWheelSpeed, 1024); //! Publish velocities as [rpm]

//! Initialize node arguments using command line
int rate_frequency;

// Command: rosrun rollo rollo_communication _rate:=1
//! Initialize node parameters from launch file or command line.
//! Use a private node handle so that multiple instances of the node
//! can be run simultaneously while using different parameters.
ros::NodeHandle private_node_handle_("~");
private_node_handle_.param("rate", rate_frequency, int(10));
// private_node_handle_.param("port", port, unsigned int(10));
private_node_handle_.param("port", port, int(10));
// private_node_handle_.param("ip", ip, IPv4(12));
// private_node_handle_.param("ip", ip, int(10));
// private_node_handle_.param("ip", ip, int(10));
// private_node_handle_.param("ip", ip, char(16));
// private_node_handle_.param("ip", ip, "192.168.0.120");
// private_node_handle_.param("ip", ip);
// private_node_handle_.param("ip", ip, cha(16));

//! Node frequency rate [Hz]
ros::Rate frequency(rate_frequency);

//! Initialize subscriber message type
geometry_msgs::Twist SubRolloTwist;

//! Initialize publisher message type
rollo::WheelSpeed PubRolloWheelSpeed;

//! Initialize variables for computing linear and angular velocity of the robot
int rv = 0;

//! Run loop
while (ros::ok()) {

	//! Send control command to Rollo
	// rv = udpSend(ip, port, Message);
	// rv = udpSend('192.168.0.120', port, Message);
	rv = udpSend("192.168.0.120", port, Message);
	if (rv)
		ROS_INFO("[Rollo][%s] Command executed", NodeName); //DB

	//! Publish encoder readings
	VelocityL = v_l;
	VelocityR = v_r;
	PubRollo.publish(PubRolloWheelSpeed);
	ROS_INFO("[Rollo][%s][Pub] L[%d] R[%d]", NodeName, VelocityL, VelocityR); //DB

	//! ROS spinOnce
	ros::spinOnce();

	//! Sleep before running loop again
	frequency.sleep();

}

return 0;

}
