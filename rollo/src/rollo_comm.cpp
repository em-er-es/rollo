/**
 * @file rollo_comm.cpp
 * @author Rabbia Asghar
 * @author Ernest Skrzypczyk
 *
 * @date 18/2/16
 *
 * @brief Communication between ROS and Rollo
 *
 * Command prototype: <b>rosrun rollo rollo_comm _rate:=10 _ip:=192.168.0.120 _port:=900 _em:=3 _square:=0 _forwardtime:=25 _turntime:=6 _squarespeed:=0.4</b>
 * \param rate: Command sending frequency of the node <!10 [Hz]>
 * \param ip: Internet protocl address of target robot <!192.168.0.120 [1]>
 * \param port: User datagram protocol taget connection port <!900 [1]>
 * \param em: Emergency time <!3 [s]>
 * \param square: Square test switch <!0 [1]>:
 *   - 0 -- Off
 *   - 1 -- Simple square test
 *   - 2 -- Double square test
 *   - n -- N-th order square test
 * \param forwardtime: Time for forward motion of robot <!25 [s]>
 * \param turntime: Time for turning the robot <!6 [s]>
 * \param squarespeed: Square test forward motion speed <!0.4 [1]>:
 *
 * Provides basic communication structure between ROS holding nodes used for localization and Rollo.\n
 * Main aspects include:
 *  - decoding linear and angular velocities provided by control node
 *  - translate and send message to Rollo
 *  - publish decoded velocities
 *  - square test or n-th order
 *  - emergency procedure
 *
 * @see https://github.com/em-er-es/rollo/
 *
 */


#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "string.h"
#include "rollo.hpp"
#include "geometry_msgs/Twist.h"
#include "rollo/WheelSpeed.h"
#include "udp.h"


/* TODO
 * Implement squarespeed
 *! Correct output
 *! Get rid of warnings for multi-character constants and overflow
 *! Define all possible messages by contructing them out of three bytes
 *! * 1: Movement/operation byte, 2: Left wheel speed, 3: Right wheel speed
 *! Run the node either at a given frequency or if there are problems with too much data for Rollo, at a rate of 60 or 30 Hz
 *!/ Make the node accepts parameters for -ip address-, udp port and node frequency
 *! Add publisher for custom message oncluding timestamped velocities [rpm] of Rollo for EKF node
 * TODO LATER
 *! Priorotize safety over control
 * Define a function that will receive UDP packets
 * Define function to parse the ip address properly
 * Determine square test turn by using negative square values
 * For holomogenic robots implement forwards-backwards square test using negative squarespeed values
// */


/**
 * @brief Global variables updated in the @p subscriberCallback() function, processed and used to send commands to the specified @p ip adress at the UDP @p port.
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

// There is no reason for these variables to be signed integers
char ip[16] = "192.168.0.120"; //!< Ip address hardcoded
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
int EmergencyTime = 3; //!< Emergency time [s]
bool FlagEmergency = 0; //!< Emergency flag

char Mode[1]; //!< Message mode description
int VelocityL, VelocityR; //!< Message velocities description

unsigned int loopcounter = 1; //!< Loop counter for debugging purpose

double RolloMax = ROLLO_SPEED_MAX; //!< Maximum speed of the Rollo
double RolloMin = ROLLO_SPEED_MIN; //!< Minimum speed of the Rollo
double RolloRange = RolloMax - RolloMin; //! Range of speed of the Rollo


/**
 * @brief Decode linear and angular velocities
 *
 * Velocities are decoded and stored as partial bytes of the UDP packet
 *
 * \param x Linear velocity
 * \param z Angular veolocity
 * \param &Message UDP message to be send to target robot
 * \param VelocityL Decoded velocity [%]
 * \param VelocityR Decoded velocity [%]
 *
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
	//! Since control node can provide abstract control values, an ideal case could be used for decoding velocities.
	//! This is discouraged, since using alternative control methods would probably have a realistic value set.\n
	if (fabs(x) < tol) { //! Linear velocity is approximately 0:

		if (z == 0) {
			//! - Complete stop
			Message[0] = 0x7b;
			VelocityL = 0;
			VelocityR = 0;
			Mode[0] = 'S';
		}
		else if (z < tol) {
			//! - Right rotation
			Message[0] = 0x7f;
			Mode[0] = 'R';
		}
		else if (z > tol) {
			//! - Left rotation
			Message[0] = 0x7e;
			Mode[0] = 'L';
		}

		Message[1] = 0x50; Message[2] = 0x10; //! - Lowest speeds for previous modes

	} else if (fabs(x) > tol) { //! Linear velocity is above tolerance threshold
		//! Determine speeds based on the position of the "dial" z:\n
		//! <pre>
		//!    |-a-|- - -*-b- - - - -|
		//!   -1   z     0           1
		//! </pre>

		double tx = x * RolloRange; //! Temporary velocity holder declaration
		double a = (z + 1.001); //! Eliminate problems with dividing through zero by adding a small number to variables
		double b = 2 - (z + 1.001);
		//! Calculate velocities according to relation expressed in linear and angular velocities ratio
		v_l = (b / a * tx) / RolloMin;
		v_r = (a / b * tx) / RolloMin;

		//! Translate velocities for Rollo:
		//! - Left wheel velocity - Second byte
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

		//! - Right wheel velocity - Third byte
		switch (abs(v_r)){
			case 0: Message[2] = 0x10; VelocityR = 6; break;
			case 1: Message[2] = 0x11; VelocityR = 12; break;
			case 2: Message[2] = 0x25; VelocityR = 19; break; //! - Temporary fix for errartic behaviour of Rollo
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
		if (x > tol) {
			Message[0] = 0x7c;
			Mode[0] = 'F';
		} else if (x < -tol) {
			Message[0] = 0x7d;
			Mode[0] = 'B';
		}

	}

/* //DB
	if (! (loopcounter % 10)) { //! Print debug messages every 10th loop, there should be an additional condition to actually print it only once
		// ROS_INFO("[Rollo][%s][DecodeVelocities] ([%f], [%f]) => [[%c][%c][%c]]", NodeName, x, z, Message[0], Message[1], Message[2]); //DB
		// ROS_INFO("[Rollo][%s][DecodeVelocities] M: %c ([%f], [%f]) => [[%d][%d][%d]]", NodeName, Mode, float(x), float(z), Message[0], Message[1], Message[2]); //DB
		ROS_INFO("[Rollo][%s][DecodeVelocities] ([%f], [%f]) => [[%d][%d][%d]]", NodeName, float(x), float(z), Message[0], Message[1], Message[2]); //DB
		// std::cout << "[Rollo][" << NodeName << "][DecodeVelocities] ([" << x << "], [" << z << "])\n" << std::endl; //DB
	}
// */

}

/**
 * @brief Assign left and right wheel velocity estimates for a given velocity command
 *
 * System odometry simulation in absence of encoder feedback using estimates from processed and analyzed data
 *
 * \param LeftVelocityEstimate Estimated velocity for left wheel determined from logs [rad/s]
 * \param RightVelocityEstimate Estimated velocity for right wheel determined from logs [rad/s]
 * \param VelocityL Velocity command decoded from control node [%]
 * \param VelocityR Velocity command decoded from control node [%]
 *
 * @warning Only velocities processed using logs are estimated.
 * 
 * @return 0
 */

int EstimateFeedbackVelocities(int VelocityL, int VelocityR, double &LeftVelocityEstimate, double &RightVelocityEstimate)
{
	//! - No movement
	if (VelocityL == 0 && VelocityR == 0) {
		LeftVelocityEstimate = 0;
		RightVelocityEstimate = 0;
	//! - Slowest movement speed -- 6%
	} else if (VelocityL == 6 && VelocityR == 6) {
		LeftVelocityEstimate = 1.380944;
		RightVelocityEstimate = 1.382213;
	//! - Slow movement speed -- 12%
	} else if (VelocityL == 12 && VelocityR == 12) {
		LeftVelocityEstimate = 9.530818;
		RightVelocityEstimate = 9.530586;
	//! - Lower medium movement speed -- 19%
	} else if (VelocityL == 19 && VelocityR == 19) {
		LeftVelocityEstimate = 16.72931;
		RightVelocityEstimate = 16.7356;
	//! - Fastest movement speed in current configuration -- 56%
	} else if (VelocityL == 56 && VelocityR == 56) {
		LeftVelocityEstimate = 32.03983;
		RightVelocityEstimate = 31.24964;
	//! - Combination of different movement speeds -- L12% & R19%
	} else if (VelocityL == 12 && VelocityR == 19) {
		LeftVelocityEstimate = 11.99827796;
		RightVelocityEstimate = 11.77803783;
	//! - Combination of different movement speeds -- L19% & R12%
	} else if (VelocityL == 19 && VelocityR == 12) {
		LeftVelocityEstimate = 11.7195477;
		RightVelocityEstimate = 11.97782072;
	//! - Combination of different movement speeds -- L31% & R38%
	} else if (VelocityL == 31 && VelocityR == 38) {
		LeftVelocityEstimate = 28.11346659;
		RightVelocityEstimate = 27.97425271;
	//! - Combination of different movement speeds -- L38% & R31%
	} else if (VelocityL == 38 && VelocityR == 31) {
		LeftVelocityEstimate = 23.6848081;
		RightVelocityEstimate = 23.79264803;
	}

	return 0;
}

/**
 * @brief Subscriber callback from control node
 *
 * Read newest velocities from control node and translate them into UDP message. Update latest message time.
 *
 * \param msg Message from control node containing linear and angular velocities
 *
 * @return 0
 */

void subscriberCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	decodeVelocities(msg->linear.x, msg->angular.z, Message, VelocityL, VelocityR); //! Update the UDP message
	lastMessageTime = ros::Time::now().toSec(); //! Update last message time
	FlagEmergency = 0; //! Reset emergency flag
}

/**
 * @brief Send UDP packets
 *
 * Send provided message using included UDP library command @p udp_client_server::udp_client.send()
 *
 * \param &ip IP address of the target robot
 * \param &port UDP port of the target robot
 * \param &message UDP message sent to robot
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
		ROS_INFO("[Rollo][%s][UDP][Mode [%c] Velocity L [%d] Velocity R [%d]", NodeName, Mode[0], VelocityL, VelocityR); //VB
	} else if (bs < 0) { //! Error handling
		ROS_INFO("[Rollo][%s][ERR] UDP packets transmission failed", NodeName); //VB
		ROS_ERROR("sendto()");
	}

	return (bs); //! Return bytes sent or error code
}

/**
 * @brief Node main
 *
 * Depending on specified parameters processes data from control node and Rollo
 * and transmits them to appropriate targets or runs a square test of n-th order
 *
 * \param rate: Command sending frequency of the node <!10 [Hz]>
 * \param ip: Internet protocl address of target robot <!192.168.0.120 [1]>
 * \param port: User datagram protocol taget connection port <!900 [1]>
 * \param em: Emergency time <!3 [s]>
 * \param square: Square test switch <!0 [1]>:
 *   - 0 -- Off
 *   - 1 -- Simple square test
 *   - 2 -- Double square test
 *   - n -- N-th order square test
 * \param forwardtime: Time for forward motion of robot <!25 [s]>
 * \param turntime: Time for turning the robot <!6 [s]>
 * \param squarespeed: Square test forward motion speed <!0.4 [1]>:
 *
 * @return 0
 */

int main(int argc, char **argv)
{
//! ## Initalization
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

//! Initialize node parameters from launch file or command line.
//! Use a private node handle so that multiple instances of the node
//! can be run simultaneously while using different parameters.
ros::NodeHandle private_node_handle_("~");

//! Node main parameters
private_node_handle_.param("rate", rate_frequency, int(25));
// IP address parsing seems to be very problematic, therefore a hardcoded assignment has been made for now
// private_node_handle_.param("ip", ip, char(16));
// private_node_handle_.param("ip", ip, "192.168.0.120");
// private_node_handle_.param("ip", ip, const char("192.168.0.120"));
// private_node_handle_.param("ip", ip, const char("192.168.0.120"));
private_node_handle_.param("port", port, int(900));
//! Emergency parameters
private_node_handle_.param("et", EmergencyTime, int(3));

//! Square test parameters
//! Default values
int square = 0;
double squarespeed = 0.4;
double turntime = 6;
double forwardtime = 25;

//! Feedback velocities in rad/s to publish
double LFeedbackVelocity = 0;
double RFeedbackVelocity = 0;

// The code below also specifies default values, however for debugging it can be disabled with comments more easily
private_node_handle_.param("square", square, int(0));
private_node_handle_.param("forwardtime", forwardtime, double(25));
private_node_handle_.param("turntime", turntime, double(6));
private_node_handle_.param("squarespeed", squarespeed, double(0.4));

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


//! ## Square test
//! Alternatively this square test could be in control node, however communication node is "closer" to Rollo
	// while ((abs(square) > 0) && ros::ok()) {
	while (square != 0 && ros::ok()) {
		loopcounter++; // Use loopcounter as square test run indicator

		//! - Print information on current run
		ROS_INFO("[Rollo][%s][Square test][%d]", NodeName, loopcounter); //DB

		//FIX Get rid of warnings:
		//warning: multi-character character constant [-Wmultichar]
		//warning: overflow in implicit constant conversion [-Woverflow]
		//FIX Implement squarespeed
		char CmdTurn[3] = {0x7b, 0x55, 0x11}; //! - Compose turn command
		//! - Check square run variable and determine turning direction
		if (square % 2) {
			// printf("square % 2 = 1\n"); //DB
			CmdTurn[0] = 0x7e;
		} else {
			// printf("square % 2 = 0\n"); //DB
			CmdTurn[0] = 0x7f;
		}
		//! For multiple runs the robot would go back and forth providing more reliable data on the actual error
		//! In ideal case even a high order square run would result in the robot being at the initial position with initial orientation
		char CmdForward[3] = {0x7c, 0x55, 0x11}; //! - Compose forward command
		ROS_INFO("[Rollo][%s][Square test][%d] Message: [%d|%d|%d]", NodeName, loopcounter, CmdForward[0], CmdForward[1], CmdForward[2]); //DB
		ROS_INFO("[Rollo][%s][Square test][%d] Turning: %d {dec}", NodeName, loopcounter, CmdTurn[0]); //DB
		//! - Set initial time
		double initialtime = ros::Time::now().toSec();
		//! -  Initialize bytes sent variable for debugging
		int bs;

		//! - Print square test parameters
		// ROS_INFO("[Rollo][%s][Square test] Starting time %g, forward time %g, turning time %g", NodeName, initialtime, forwardtime, turntime); //DB
		ROS_INFO("[Rollo][%s][Square test][%d] Forward time %g, forward speed %f, turning time %g", NodeName, loopcounter, squarespeed, forwardtime, turntime); //VB

		for (int i = 0; i < 4; i++) { //! ### Main square loop

			//! Moving forward
			ROS_INFO("[Rollo][%s][Square test][%d] Moving forward: %g [s] @ %f", NodeName, loopcounter, forwardtime, squarespeed); //VB
			for (int j = 0; j < 3; j++) bs += udpClient.send(CmdForward, nb); //! Send command 3 times
			ros::Duration(forwardtime, 0).sleep(); //! Wait for the specified time to move forward
			// Alternatively use a while loop here and update the command either at the rate of the loop or other

			//! Turning
			ROS_INFO("[Rollo][%s][Square test][%d] Turning: %g [s]", NodeName, loopcounter, turntime); //VB
			for (int j = 0; j < 3; j++) bs += udpClient.send(CmdTurn, nb); //! Send command 3 times
			ros::Duration(turntime, 0).sleep(); //! Wait for the specified time to turn
			// Alternatively use a while loop here and update the command either at the rate of the loop or other

		} //! ### Main square loop end

			double finishtime = ros::Time::now().toSec(); //! Update run finish time
			//! Print duration time
			ROS_INFO("[Rollo][%s][Square test][%d] Finished: %g [s]", NodeName, loopcounter, abs(finishtime - initialtime)); //DB

		//! Check square loop condition
		if (square > 1) {
			//! Turn around
			ROS_INFO("[Rollo][%s][Square test][%d] Turn around", NodeName, loopcounter); //VB
			for (int j = 0; j < 3; j++) bs += udpClient.send(CmdTurn, nb); //! Send command 3 times
			ros::Duration(2 * turntime, 0).sleep(); //! Wait for twice the specified time to turn around

			//! Update square run counter and check for exit condition
			square--;
		} else {
			//! Stop
			ROS_INFO("[Rollo][%s][Square test][%d] Stop", NodeName, loopcounter); //VB
			for (int j = 0; j < 10; j++) bs += udpClient.send(MessageEmergencyStop, nb); //! Send stop command 10 times

			//! Update square run counter and check for exit condition
			return 0;
		}

		// if (square > 1) square--;
		// else return 0; //! Update square run counter and check for exit condition
		// if (square > 1) square--;
		// else if (square < 1) square++;
		// else return 0;
		// Alternative approaches for variable check and exit
		// if (! square % 2) square--; else return 0;
		// square--;
		// if (!square) return 0;
	}
//! ## Square test end

//! ## Main loop
while (ros::ok()) {

	//! - Send control command to Rollo
	rv = udpSend(ip, port, Message);

	//! ### Emergency procedure
	//! - Check if emergency condition has been met:
	currentTime = ros::Time::now().toSec();
	if (abs(lastMessageTime - currentTime) > abs(EmergencyTime)) {
		//! - Print emergency message
		ROS_INFO("[Rollo][%s][Emergency stop][Emergency time: %d] Apparent loss of connection to control node!", NodeName, EmergencyTime); //DB
		//! - Conduct emergency stop
		for (int i = 0; i < 10; i++) //! Send emergency message 10 times
			rv = udpSend(ip, port, MessageEmergencyStop);
		//! - Hard condition emergency procedure\n
		//! Exit with an error code if hard condition is set by using negative values for emergency time
		if (EmergencyTime < 0) return 128;
		//! - Soft condition emergency procedure\n
		FlagEmergency = 1; //! Set emergency flag
		while (FlagEmergency && ros::ok()) { //! Empty procedure sequence if emergency flag is raised

			//! - ROS spinOnce
			ros::spinOnce();

			//! - Sleep before running loop again
			frequency.sleep();

		}
	}
	//! ### Emergency procedure end


	//! - Estimate feedback velocities
	EstimateFeedbackVelocities(VelocityL, VelocityR, LFeedbackVelocity, RFeedbackVelocity);

	//! - Compose message to be published
	//!   - Assign timestamp
	//!   - Assign estimated feedback velocities
	PubRolloWheelSpeed.header.stamp = ros::Time::now();

	// PubRolloWheelSpeed.wheelspeedleft = v_l;
	// PubRolloWheelSpeed.wheelspeedright = v_r;
	PubRolloWheelSpeed.wheelspeedleft = LFeedbackVelocity;
	PubRolloWheelSpeed.wheelspeedright = RFeedbackVelocity;

	//! - Publish message
	PubRollo.publish(PubRolloWheelSpeed);

	//! - Print published message
	ROS_INFO("[Rollo][%s][Pub] Message: [%d|%d|%d], L[%d], R[%d]", NodeName, Message[0], Message[1], Message[2], VelocityL, VelocityR); //DB
	// ROS_INFO("[Rollo][%s][Pub] TS[%d.%d] L[%d] R[%d]", NodeName, PubRolloWheelSpeed.header.stamp.sec, PubRolloWheelSpeed.header.stamp.nsec, PubRolloWheelSpeed.wheelspeedleft, PubRolloWheelSpeed.wheelspeedright); //DB

	//! ROS spinOnce
	ros::spinOnce();

	//! Sleep before running loop again
	frequency.sleep();

	//! Increase loopcounter
	loopcounter++;
}
//! ## Main loop end

return 0;

}
