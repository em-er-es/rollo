/**
 * @file rollo_comm.cpp
 * @author Rabbia Asghar, Ernest Skrzypczyk
 * @date 18/2/16
 * @brief Communication between nodes and Rollo
 *
 *
 * @see https://github.com/em-er-es/rollo/
 */

// Naming conventions:
// Global and important variables: CapitalLettersFullName
// Local, temporary and irrelevant variables: shortlowercase
// Functions priority: CapitalLettersFunctions
// Functions conventional: firstLowerLetterFunction
// Functions and variables special: _FullDescription

#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "rollo_nodes.h"

// /*
#include "udp.h"
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"



void Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
udp_client_server::udp_client udp_c("130.251.13.185",5900);
char buff_to_send[3]={0x7d,0x59,0x31}; 
udp_c.send(buff_to_send,3);
printf("udp_packet_Sent!! \n");
sleep(1);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("test", 1, Callback);
  ros::spin();
}
// */

/**
 * @brief GLOBVAR
 * 
 */

double x, y, theta;
double x_mm, y_mm, theta_deg;


/**
 * @brief FUNDES
 *
 * FUNDES
 * @return VALUEDES
 * @see DOCURL
 */
 
void fun()
{
	int a = 0;

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0)
		ROS_INFO("[Rollo][getch] no_key_pressed");
	else
		{
		read(filedesc, &buff, len );
		ROS_INFO("[Rollo][Main] Key pressed %c \n", buff);
		}

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}


/**
 * @brief Main function
 *
 * 
 * @return 0
 */
int main(int argc, char **argv)
{
//! Initialize node
ros::init(argc, argv, "rollo_name"); // LOOKUP nodes.txt
ros::start(); // ROS node initialization

//! Initialize nodehandle
ros::NodeHandle RolloControlNode;

//! Initialie publisher and define topic and message queue size for the publisher
ros::Publisher RolloTwist = RolloControlNode.advertise<geometry_msgs::Twist>("/Rollo/cmd_vel", 1024);

//! Initialize node arguments using command line
int rate_frequency;

// Sample command: rosrun rollo rollo_control_node _rate:=1 
//! Initialize node parameters from launch file or command line.
//! Use a private node handle so that multiple instances of the node can be run simultaneously
//! while using different parameters.
ros::NodeHandle private_node_handle_("~");
private_node_handle_.param("rate", rate_frequency, int(10));

//! publishing rate in units of Hz
ros::Rate frequency(rate_frequency); 

//! Initialize publisher message type
geometry_msgs::Twist PubRolloTwist;

//! Initialize variables for computing linear and angular velocity of the robot
double x = 0;
double th = 0;
double speed = 0.5;
double turn = 1;


//! Run loop
while (ros::ok())
	{
		//! read if any keyboard key is pressed
		int c = 0;
		c=getch();

		//! decode key pressed 
		if (c != 0) decode_key(c, x, th, speed, turn);

		//! compute linear and angular velocity
		PubRolloTwist.linear.x = x*speed; 
		PubRolloTwist.angular.z = th*turn;

		//! publish Rollo Twist according to the velocities defined
		ROS_INFO("[Rollo][Pub] Linear Speed %f Angular Speed %f \n", PubRolloTwist.linear.x, PubRolloTwist.angular.z);
		RolloTwist.publish(PubRolloTwist);

		//! ros spinOnce
		ros::spinOnce();

		//!sleep before running loop again 
		frequency.sleep();

	}

return 0;
}
