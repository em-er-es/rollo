/**
 * @file rollo_preprocessor.cpp
 * @author Rabbia Asghar
 * @author Ernest Skrzypczyk
 *
 * @date 16/2/16
 *
 * @brief Preprocessor for Rollo measurement using Mocap OptiTrack motion capture data
 *
 * Command prototype: <b>rosrun rollo rollo_preprocessor _rate:=25 _samplesize:=4 _sampling:=0</b>
 * \param rate: Sampling frequency of the node <!25 [Hz]>
 * \param samplesize: Number of elements that are averaged/subsampled <!4 [1]>
 * \param sampling: Selects if the raw data should be subsampled after a certain delay or averaged over a certain period <!0 [1]>
 *    - sampling 0 sets subsampling
 *    - sampling !0 sets averaging
 *
 * Filter the raw data from optitrack motion capture system and
 * publish it along with time stamp for modeling of odometry and
 * the measurement in Kalman filter
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "rollo/Pose2DStamped.h"
#include "tf/tf.h"
#include <sstream>
#include <iostream>
#include "rollo.hpp"


/* TODO
 * Instead of using two parameters for samplesize and sampling/averaging, use one, where negative values are used for the least common operation and positive for the most common
 * FIX DOXYGEN documentation
// */


/**
 * @brief Global variables updated in the SubscriberCallback function, processed and published.
 *
 */

//! Node name using console codes
char NodeName[20] = C1 PP CR; // The size is necessary for the GNU/Linux console codes //~COLOR
// char NodeName[20] = PP; // The size is necessary for the GNU/Linux console codes //~COLOR

//! Absolute coordinates
double x, y, theta;
//! Absolute coordinates in various units
double x_mm, y_mm, theta_deg;

//! Topics
//! Topic for motion capture data
char TopicMotionCapture[64] = TOPIC_PREP_MC;
//! Topic for position and orientation, stamped
char TopicPose2DStamped[64] = TOPIC_PREP_P2DT;


/**
 * @brief Subscriber callback
 *
 * Subscribe to motion capture data from @ref mocap_optitrack node and read position and orientation from Optitrack node.
 *
 * @param msg Message generated by @ref mocap_optitrack node in format:
 * - Position x [m]
 * - Position y [m]
 * - Orientation [rad]
 *
 * @see https://github.com/ros-drivers/mocap_optitrack
 *
 * @return NULL
 */

void subscriberCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
	//! Acquisition:
	//! - Raw x coordinate [m]
	x = msg->x;
	x_mm = 1000 * x;
	//! - Raw y coordinate [m]
	y = msg->y;
	y_mm = 1000 * y;
	//! - Raw theta [rad]
	theta = msg->theta;
	//! Conversion into degrees in the range 0 to 360 degress
	theta_deg = theta / PI * 180 + 180;
	//! Print message with acquired data
	ROS_INFO("[Rollo][%s][Sub][X [mm], Y [mm], Theta [deg]]: %f, %f, %f", NodeName, x_mm, y_mm, theta_deg);
}


/**
 * @brief Node main
 *
 * Initialize variables, nodehandle, subscribe to motion capture data from @ref mocap_optitrack node and publish position and orientation after processing with time stamp.
 * The position and orientation are published along with timestamp to topic /Rollo/preprocessor/pose2dstamped in format custom defined message, rollo::Pose2DStamped .
 * @return 0
 */
int main(int argc, char **argv)
{

//! ## Initialization
ros::init(argc, argv, "rollo_preprocessor"); //! Name of the preprocessor node
ros::start(); // ROS internal function, neccessary to be called

//! - Nodehandle for subscriber and publisher
ros::NodeHandle RolloPreprocessorNode;

//! - Subscriber
ros::Subscriber PoseSub = RolloPreprocessorNode.subscribe(TopicMotionCapture, 1024, subscriberCallback);

//! - Publisher initialization with topic, message format and queue size definition
ros::Publisher RolloPub = RolloPreprocessorNode.advertise<rollo::Pose2DStamped>(TopicPose2DStamped, 1024);

//! - Node arguments using command line
int rate_frequency;
int samplesize;
int sampling; //! Sampling is either done using subsampling (0) or simple averaging (1)

//! - Initialize node parameters from launch file or command line.
//! Use a private node handle so that multiple instances of the node can be run simultaneously
//! while using different parameters.
ros::NodeHandle private_node_handle_("~");
private_node_handle_.param("rate", rate_frequency, int(25));
private_node_handle_.param("samplesize", samplesize, int(4));
private_node_handle_.param("sampling", sampling, int(0));

//! - Publishing rate [Hz]
ros::Rate frequency(rate_frequency);

//! - Publisher variables for conventional messages
std_msgs::String Message;
std::stringstream StringStream;

//! - Message type
std_msgs::String PubRolloPosition;

//! - Publisher variables for processing
double sum_x = 0;
double sum_y = 0;
double sum_theta = 0;

double average_x = 0;
double average_y = 0;
double average_theta = 0;

//! - Initialize variable to publish message
rollo::Pose2DStamped PubRolloPositionPose2dStamped;
PubRolloPositionPose2dStamped.header.frame_id = '1'; // Global frame

//! - Loop counter holder
unsigned int loopcounter = 0;
//! - Loop condition variable
int loopcondition = 1; // For while(1) loop


//! ## Main loop
do {
	sum_x += x;
	sum_y += y;
	sum_theta += theta;

	if (loopcounter >= samplesize)
	{
		average_x = sum_x / samplesize;
		average_y = sum_y / samplesize;
		average_theta = sum_theta / samplesize;

		//StringStream << "[Rollo][" << NodeName << "][X, Y, Theta]: " << x_mm << ", " << y_mm << ", " << theta_deg << "\n";
		//StringStream << "[Rollo][" << NodeName << "][X, Y, Theta]: " << sum_x_mm << ", " << sum_y_mm << ", " << sum_theta_deg << "\n";
		//StringStream << "[Rollo][" << NodeName << "]["<<samplesize<<"][X, Y, Theta]: " << average_x_mm << ", " << average_y_mm << ", " << average_theta_deg << "\n";
		//StringStream << "[Rollo][" << NodeName << "][X, Y, Theta]: " << average_x_mm << ", " << average_y_mm << ", " << average_theta_deg << "\n";
		//PubRolloPosition.data = StringStream.str();

		//! - Prepare data for publishing
		PubRolloPositionPose2dStamped.pose.x  = average_x;
		PubRolloPositionPose2dStamped.pose.y  = average_y;
		PubRolloPositionPose2dStamped.pose.theta  = average_theta;
		PubRolloPositionPose2dStamped.header.stamp = ros::Time::now();

		//! - Publish
		RolloPub.publish(PubRolloPositionPose2dStamped);

		//! - Reset variables
		loopcounter = 0;
		sum_x  = 0;
		sum_y  = 0;
		sum_theta = 0;

		//! - For subsampling sleep for time defined by rate and then read the states from the @ref subscriberCallback() without usleep() delay
		if (sampling == 0) frequency.sleep();
	}


	// ROS_INFO("[Rollo][%s][Debug][Counter]: %d", NodeName, loopcounter); //DB
	ros::spinOnce();
	if (! ros::ok()) loopcondition = 0; //TEST seems unnecessary

	//! For averaging sleep for time defined by rate before reading states from the subscriberCallback()
	if (sampling != 0) frequency.sleep();
	//! - Increase loop counter
	loopcounter++;

} while (loopcondition);
//! ## Main loop end

return 0;
}
