#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/tf.h"
#include <sstream>
#include <iostream>
// #include <Python/Python.h>

// ros::Publisher RolloPub;

/*
void subscriberCallback(const geometry_msgs::Pose::ConstPtr& PoseMsg)
{
 geometry_msgs::Pose MsgSent;
 //turtlesim::Pose test= *PoseMsg;

 //std::cout << test.x << std::endl;
 if ((PoseMsg->x >= 1.0)&(PoseMsg->x<=9.5))
	{
	MsgSent.linear.y = 0.0;
	MsgSent.linear.x = 1.0;
	MsgSent.angular.z=0.0;
	}

	else if ((PoseMsg->x > 9.5)&(PoseMsg -> theta <3.135))
	{
	MsgSent.linear.y = 0.0;
	MsgSent.linear.x = 0.1;
	MsgSent.angular.z=0.3;
	}

 else if ((PoseMsg->x <1.5)&(PoseMsg -> theta >0.005))
	{
	MsgSent.linear.y = 0.0;
	MsgSent.linear.x = 0.1;
	MsgSent.angular.z=-0.3;
	}

	else if ((PoseMsg->x > 9)&(PoseMsg -> theta >3.135))
	{
	MsgSent.linear.y = 0.0;
	MsgSent.linear.x = 1.0;
	MsgSent.angular.z=0.0;
	}

 else if ((PoseMsg->x	< 1.5)&(PoseMsg -> theta <0.005))
	{
	MsgSent.linear.y = 0.0;
	MsgSent.linear.x = 1.0;
	MsgSent.angular.z=0.0;
	}

	RolloPub.publish(MsgSent);
	
	if ((PoseMsg->y >8.9))
	{
ros::shutdown();
	}
}
*/

// Global variables
float x, y, theta;
float x_mm, y_mm, theta_rad;

// Function definitions
// void subscriberCallback(const std_msgs::String::ConstPtr& msg) {
// void subscriberCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
void subscriberCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
	// float x = msg->x, y = msg->y, theta = msg->theta;
	x = msg->x;
	x_mm = 1000 * x;
	y = msg->y;
	y_mm = 1000 * y;
	theta = msg->theta;
	theta_rad = theta / 3.14159265359 * 180 + 180;
	// ROS_INFO("[Rollo][Sub][X, Y, Theta]: %f, %f, %f", msg->x, msg->y, msg->theta);
	// ROS_INFO("[Rollo][Sub][X, Y, Theta]: %f, %f, %f, %f", msg->x, msg->y, msg->theta, theta_rad);
	ROS_INFO("[Rollo][Sub][X, Y, Theta]: %f, %f, %f", x_mm, y_mm, theta_rad);
	// return (x, y, theta);
}


int main(int argc, char **argv)
{
// Initialization
ros::init(argc, argv, "rollo_pubsub"); // name of the converter node
ros::start(); // Necessary to be called
// ros::init(argc, argv, "rollo_node"); // name of the converter node
ros::Rate frequency(10); // 10 Hz

// Main function variables
// float x, y, theta;

// Nodes
ros::NodeHandle NodeSubscribe;
ros::NodeHandle NodePublish;

// Subscriber
ros::Subscriber PoseSub = NodeSubscribe.subscribe("/Optitrack_Rollo/ground_pose", 1024, subscriberCallback);
// Check if subscribed correctly otherwise exit
// ros::Publisher RolloPub = NodePublish.advertise<geometry_msgs::Pose>("/rosout", 1024);
// ros::Publisher RolloPub = NodePublish.advertise<geometry_msgs::Pose2D>("/Rollo/pose2d", 1024);
ros::Publisher RolloPub = NodePublish.advertise<std_msgs::String>("/Rollo/pose", 1024);

// Publisher
int condition = 1;
int loopcounter = 0;

// Standard messages
// std_msgs::String Message;
// std::stringstream StringStream;
// fprintf(StringStream, "%d -- %d", loopcounter, condition);
// sprintf(StringStream, "%d -- %d", loopcounter, condition);
// sprintf(StringStream, loopcounter + " -- " + condition);

// Geometry messages
// URL: http://docs.ros.org/api/geometry_msgs/html/msg/Pose2D.html
// # This expresses a position and orientation on a 2D manifold.
// float64 x
// float64 y
// float64 theta
// geometry_msgs::Pose2D PubRolloPosition; // Declaration of type of message 
std_msgs::String PubRolloPosition; // Declaration of type of message 
std::stringstream StringStream;
float PubPose2dX = 0;
float PubPose2dY = 0;
float PubPose2dTheta = 0;
int samplesmooth = 10;
float PO_i[3][samplesmooth] = {x, y, theta};
float PO_s[3] = {x, y, theta};
// geometry_msgs::Printer StringStream;
// StringStream << loopcounter << " -- " << condition;
// StringStream << "[Rollo][Current position and orientation (Loop, X, Y, Theta)]: " << PubPose2dX << ", " << PubPose2dY << ", " << PubPose2dTheta;
// StringStream << "[Rollo][X, Y, Theta]: " << PubPose2dX << ", " << PubPose2dY << ", " << PubPose2dTheta << "\n";
// StringStream << "[Rollo][X, Y, Theta]: " << PubPose2dX << ", " << PubPose2dY << ", " << PubPose2dTheta;
// PubRolloPosition.data = StringStream.str();

do {
// StringStream << "[Rollo][Loop: (X, Y, Theta)]: " << loopcounter << ":  " << PubPose2dX << ", " << PubPose2dY << ", " << PubPose2dTheta;
// ROS_INFO("%s", PubRolloPosition.data.c_str());
// ROS_INFO("[Rollo][X, Y, Theta]: %f, %f, %f", PubPose2dX, PubPose2dY, PubPose2dTheta);
// ROS_INFO("[Rollo][X, Y, Theta]: %d, %f, %f, %f", loopcounter, PubPose2dX, PubPose2dY, PubPose2dTheta);
// float PO_i[3] = subscriberCallback;
// float PO_f[3] = subscriberCallback;
// frequency.sleep();
// float PO_f[3] = {x, y, theta};
// PubPose2dX = PO_i[0];
// PubPose2dY = PO_i[1];
// PubPose2dTheta = PO_i[2];
// ROS_INFO("%d", loopcounter);
//	ros::spin();
PO_i[0][loopcounter] = x;
PO_i[1][loopcounter] = y;
PO_i[2][loopcounter] = theta;

if (loopcounter > 9){
	for (int j = 0; j++; j < 3){
		ROS_INFO("[Rollo][Debug][PO_i]: %f", PO_i[j]);
		for (int i = 0; i++; i < samplesmooth){
			PO_s[j] = PO_s[j] + PO_i[j][i];
			ROS_INFO("[Rollo][Debug][PO_i]: %f", PO_i[j][i]);
			}
		PO_s[j] /= samplesmooth;
		}
	ROS_INFO("[Rollo][Debug][PO_s]: %f", PO_s);
	// ROS_INFO("[Rollo][Pub][X, Y, Theta]: %f, %f, %f", PubPose2dX, PubPose2dY, PubPose2dTheta);
	ROS_INFO("[Rollo][Pub][X, Y, Theta]: %f, %f, %f", PO_s[0], PO_s[1], PO_s[2]);
	loopcounter = 0;
}

RolloPub.publish(PubRolloPosition);
ros::spinOnce();

// ROS_INFO("[Rollo][Debug][Counter]: %d", loopcounter);

if (! ros::ok()) condition = 0;
frequency.sleep();
loopcounter++;
} while (condition);
//end while loop
// ros::shutdown();
return 0;
}
