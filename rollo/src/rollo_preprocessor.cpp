/**
 * @file rollo_node.cpp
 * @author Rabbia Asghar, Ernest Skrzypczyk
 * @date 9 16/2/16
 * @brief Preprocessor for Rollo measurement using Mocap OptiTrack motion capture data
 *
 * Filter the raw data from optitrack motion capture system and 
 * publish it for modeling of odometry and the measurement in Kalman Filter
 * 
 * Command: rosrun rollo rollo_node _rate:=1 _samplesize:=5 _sampling:=0
 *  _rate: Sampling frquency of the node, default value is 1
 *  _samplesize: Number of elements that are averaged/subsampled, default value is 10
 *  _sampling: Selects if the raw data should be subsampled after a certain delay or averaged over a certain period
 *  _sampling 0 sets subsampling and any number other than 0 sets averaging, default is set to subsampling
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/tf.h"
#include <sstream>
#include <iostream>
#include "rollo_nodes.h"


/**
 * @brief Global variables updated in the SubscriberCallback function and later used in main to process and publish.
 * 
 */
 
double x, y, theta;
double x_mm, y_mm, theta_deg;


/**
 * @brief SubscriberCallback function
 *
 * This is a gloabal function that subscribes to the topic 'ground_pose' of the Optitrack node.
 * It reads position and orientation from Optitrack node.
 * @param msg message generated by Optitrack node contains position in x and y in metres and orienatation in radians.
 * @return NULL
 */

void subscriberCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
	x = msg->x; // Raw x coordinate [m]
	x_mm = 1000 * x;
	y = msg->y; // Raw y coordinate [m]
	y_mm = 1000 * y;
	theta = msg->theta; // Raw theta [rad]
	theta_deg = theta / 3.14159265359 * 180 + 180; // Conversion into degrees in the range 0 to 360 degress
	// ROS_INFO("[Rollo][Sub][X, Y, Theta]: %f, %f, %f", msg->x, msg->y, msg->theta);
	// ROS_INFO("[Rollo][Sub][X, Y, Theta]: %f, %f, %f, %f", msg->x, msg->y, msg->theta, theta_deg);
	ROS_INFO("[Rollo][Sub][X, Y, Theta]: %f, %f, %f", x_mm, y_mm, theta_deg);
}


/**
 * @brief Main function
 *
 * Initializes variables, nodehandle, subscribes to Optitack Ground_pose and publishes position and orientation after processing.
 * It accepts 3 arguments from command line: rate, samplesize, sampling.
 * The position and orientation are published to topic /Rollo/pose in format geometry_msgs::Pose2D
 * @return 0
 */
int main(int argc, char **argv)
{

//! Initialization 
ros::init(argc, argv, "rollo_preprocessor"); // Name of the preprocessor node
ros::start(); // ROS internal function, neccessary to be called

//! Nodehandle for subscriber and publisher
ros::NodeHandle RolloNode;

//! Subscriber
ros::Subscriber PoseSub = RolloNode.subscribe("/Optitrack_Rollo/ground_pose", 1024, subscriberCallback);

//! Publisher initialization with topic, message format and queue size definition
ros::Publisher RolloPub = RolloNode.advertise<geometry_msgs::Pose2D>("/Rollo/pose", 1024);

//! Node arguments using command line
int rate_frequency;
int samplesize;
int sampling; // Sampling is either done using subsampling (0) or simple averaging (1) 

// Command: rosrun rollo rollo_node _rate:=1 _samplesize:=5 _sampling:=0
//! Initialize node parameters from launch file or command line.
//! Use a private node handle so that multiple instances of the node can be run simultaneously
//! while using different parameters.
ros::NodeHandle private_node_handle_("~");
private_node_handle_.param("rate", rate_frequency, int(1));
private_node_handle_.param("samplesize", samplesize, int(4));
private_node_handle_.param("sampling", sampling, int(0));

//! Publishing rate [Hz]
ros::Rate frequency(rate_frequency); 

//! Publisher variables for conventional messages
std_msgs::String Message;
std::stringstream StringStream;

std_msgs::String PubRolloPosition; // Declaration of message typey

//! Publisher variables for processing
double sum_x_mm = 0;
double sum_y_mm = 0;
double sum_theta_deg = 0;

double average_x_mm = 0;
double average_y_mm = 0;
double average_theta_deg = 0;

int loopcounter = 0;
int loopcondition = 1; // For while(1) loop


//! Loop
do {
	sum_x_mm += x_mm;
	sum_y_mm += y_mm;
	sum_theta_deg += theta_deg;


	if (loopcounter >= samplesize)
	{
		average_x_mm = sum_x_mm / samplesize;
		average_y_mm = sum_y_mm / samplesize;
		average_theta_deg = sum_theta_deg / samplesize;

		//StringStream << "[Rollo][X, Y, Theta]: " << x_mm << ", " << y_mm << ", " << theta_deg << "\n";
		//StringStream << "[Rollo][X, Y, Theta]: " << sum_x_mm << ", " << sum_y_mm << ", " << sum_theta_deg << "\n";
		//StringStream << "[Rollo]["<<samplesize<<"][X, Y, Theta]: " << average_x_mm << ", " << average_y_mm << ", " << average_theta_deg << "\n";
		//StringStream << "[Rollo][X, Y, Theta]: " << average_x_mm << ", " << average_y_mm << ", " << average_theta_deg << "\n";
		//PubRolloPosition.data = StringStream.str();

		//! Prepare data for publishing
		geometry_msgs::Pose2D PubRolloPositionPose2d;
		PubRolloPositionPose2d.x = average_x_mm;
		PubRolloPositionPose2d.y = average_y_mm;
		PubRolloPositionPose2d.theta = average_theta_deg;

		//! Publish
		RolloPub.publish(PubRolloPositionPose2d);

		//! Reset variables
		loopcounter = 0;
		sum_x_mm  = 0;
		sum_y_mm  = 0;
		sum_theta_deg = 0;

		//! For subsampling sleep for time defined by rate and then read the states from the subscriber callback() without sleep() delay 
		if (sampling == 0) frequency.sleep();
	}


	// ROS_INFO("[Rollo][Debug][Counter]: %d", loopcounter); //DB
	ros::spinOnce();
	if (! ros::ok()) loopcondition = 0;

	//! For averaging sleep for time defined by rate before reading states from the subscriber callback()
	if (sampling != 0) frequency.sleep();
	loopcounter++;

} while (loopcondition);
//! End while loop

// ros::shutdown();
return 0;
}
