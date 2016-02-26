/**
 * @file rollo_ekf.cpp
 * @author Rabbia Asghar
 * @author Ernest Skrzypczyk
 *
 * @date 20/2/16
 *
 * @brief EKF implementation for localisation of the robot
 *
 * Command prototype: <b>rosrun rollo rollo_ekf _rate:=1</b>
 * @param rate: Sampling frequency of the node <!1 [Hz]>
 *
 * Based on input from communication node in form of control commands and measurement from preprocessor node,
 * extended Kalman filter implementation estimates of states for localization and publishes estimated states with covariance. Additional infromation in form of odometry based state estimate are also published for easier analysis of the filter results.
 *
 * Localization of the robot consists of 3 states:
 *  - Position (x, y)
 *  - Orientation (Theta)
 *
 * Timing for EKF update is inspired from @ref Robot Pose EKF (robot/pose/ekf) package available for ROS:\n
 * Timings and data at those specific time instants are synchronized in such a manner, that the latest
 * measurements with newer timestamps are interpolated to one and the same timestamp, when all necessary
 * data is available. This allows for a relative comparison of available data, even though an additional
 * error is introduced through interpolating.
 * @see http://wiki.ros.org/robot_pose_ekf
 *
 * Kalman filter equations were first implemented in MATLAB, then translated into C++, compared and verified with previous results.
 *
 * @see https://github.com/em-er-es/rollo/
 *
 */


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/tf.h"
#include "rollo/Pose2DStamped.h"
#include "rollo/WheelSpeed.h"
#include "rollo/EKF.h"
#include <sstream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include "rollo.hpp"


/* TODO
 *! Complete TODO points can be marked with ! like this one
 * FIX DOXYGEN formatting
 * There is a difference between commenting in code and for the documentation, at least take a look at refman.pdf
 * This needs to be uniform accross all nodes
 * FIX DOXYGEN formatting
 * Should the frequency be increased to 4 Hz or more?
 * FIX DOXYGEN formatting
 * FIX DOXYGEN formatting
 * Add more doxygen comments and appropriate description
 * Double check if std:cout and ROS_INFO do not collide or produce unnecessary output, std::cout is fine for debug, just update comment switches for those lines
 * Topics should be held in one place at the beginning of the code or maybe even better in the header file TOPIC_nodename#number, so TOPIC_COMM1 or TOPIC_COMM_WS for wheel speed
 * I suggest using descriptive names for node references, so communication node instead of rollo_comm, because that might change, the main name less likely
 * Massive clean up of comments in code and description of functions
 * Begin comments with CAPITAL LETTER
 * DO NOT use absolute references in the documentation
 * Check how the documentatin actually looks like
 * Use short straight to the point impersonal description
 * Get rid of double, triple and multiple spaces in the code, one space as whitespace seperator, not more
 * Double check indents, main stays without any, while loops have indents just as much as do if, else, switches and so on -- Use CTRL-B to jump between braces
 * Write command;[SPACE]//DB not command;//DB -- this is much more difficult to parse
 * For unit use [SI_UNIT] format in comments and even output
 * Use double line break between sections, like global variables, function declarations
 * Use one line breaks between sections in functions and between specific elements of a section
 * Use spacing between parameters and operators like *, /, +, -, sometimes even braces or brackets -- Make the code more readable
 * TODO LATER
 * Create appropriate comment switches
// */


/**
 * @brief Global variables updated in the SubscriberCallback functions.
 *
 * Initialize custom defined messages for meaurement and odometry.
 * Measurement message includes Pose2D along with timestamp.
 * Odometry message includes timestamp and angular velocities for left and right wheel.
 * Initialize variables that save timestamps from both measurement and odometry
 * subscribers in double (float64 in message format).
 */

rollo::Pose2DStamped zPose2DStamped;
double zTimeSecs = 0;

rollo::WheelSpeed Odometry;
double OdometryTimeSecs = 0;

//! Node name using console codes
char NodeName[20] = C1 KF CR; // The size is necessary for the GNU/Linux console codes //COLOR
// char NodeName[20] = KF; // The size is necessary for the GNU/Linux console codes //COLOR

// Topics
//! Topic for extended Kalman filter results with all three estimated states and covariance matrix, stamped
char TopicEKF[64] = TOPIC_EKF;
//! Topic for wheel speed containing the actual speed of wheel, preferably extracted from encoders or if not available by using a lookup table
char TopicWheelSpeed[64] = TOPIC_COMM_WS;
//! Topic for position and orientation stamped from preprocessor node
char TopicPose2DStamped[64] = TOPIC_PREP_P2DT;


/**
 * @brief SubscriberCallbackMeasurement
 *
 * Subscribe to the topic '/Rollo/preprocessor/pose2dstamped' of the preprocessor node.
 * Read filtered position and orientation of the robot and timestamp.
 * Update global variables @p zPose2DStamped and @p zTimeSecs for use in EKF update.
 *
 * @param msg - custom defined message (preprocessor node).
 *
 * @return NULL
 */

void subscriberCallbackMeasurement(const rollo::Pose2DStamped msg) {

	zPose2DStamped = msg;
	Eigen::Vector3d z(0, 0, 0);
	unsigned int zSeq = 0;

	//! Read new message
	z(0) = msg.pose2d.x;
	z(1) = msg.pose2d.y;
	z(2) = msg.pose2d.theta;
	zSeq = msg.header.seq;
	zTimeSecs = msg.header.stamp.toSec();

//	ROS_INFO("[Rollo][%s][Sub][z(0), z(1), z(2)]: %f, %f, %f", NodeName, z(0), z(1), z(2));
	//ROS_INFO("[Rollo][%s][Sub][zseq, zstamp]: %d, %f", NodeName, zSeq, zTimeSecs);

}

/**
 * @brief SubscriberCallbackControlInput
 *
 * Subscribe to the topic @var TopicWheelSpeed of the communication node.
 * Read wheel speed for both left and right in radians and timestamp.
 * Update global variables Odometry and OdometryTimeSecs for use in EKF update.
 *
 * @param msg - custom defined message (communication node).
 *
 * @return NULL
 */

void subscriberCallbackControlInput(const rollo::WheelSpeed msg) {

	//! Read new message
	Odometry = msg;
	unsigned int OdometrySeq = 0;

	OdometrySeq = msg.header.seq;
	OdometryTimeSecs = msg.header.stamp.toSec();

	// RightWheelAngularVelocity = msg.wheelspeedright;
	// LeftWheelAngularVelocity = msg.wheelspeedleft;

//	ROS_INFO("[Rollo][%s][Sub][R_AngVelocity, L_AngVelocity]: %f, %f", NodeName, Odometry.wheelspeedright, Odometry.wheelspeedright);
	//ROS_INFO("[Rollo][%s][Sub][OdomSeq, OdomStamp]: %d, %f", NodeName, OdometrySeq, OdometryTimeSecs);

}

/**
 * @brief Linear interpolation of values from odometry
 *
 * This function performs linear interpolation of right and left wheel speed for a given time instant.
 * The time for which the odometry values are computed is defined by @p EKFfilterTimeSecs.
 *
 * @param Odometryold contains left and right wheel speed and timestamp read at previous instant when EKF was updated.
 * @param Odometrynew contains left and right wheel speed and timestamp read currently.
 * @param EKFfilterTimeSecs is the time instant for which the EKF update need to be performed and odometry values need to be computed.
 *
 * @return rollo::WheelSpeed, contains left and right wheel speed [rad/s] computed for time instant given by @p EKFfilterTimeSecs using linear interpolation.
 */

//CRF
rollo::WheelSpeed interpolateOdometry( rollo::WheelSpeed Odometryold, rollo::WheelSpeed Odometrynew, double EKFfilterTimeSecs ) {

	rollo::WheelSpeed interpolated;

	double OdometryOldTimeSecs = Odometryold.header.stamp.toSec();
	double OdometryNewTimeSecs = Odometrynew.header.stamp.toSec();

//CRC
	//y = y0 + ((y1-y0)/(t1-t0))(t-t0)
	//TODO Maybe you could use temporary variables for more readability?
	interpolated.wheelspeedright = Odometryold.wheelspeedright + ((Odometrynew.wheelspeedright - Odometryold.wheelspeedright) * (EKFfilterTimeSecs - OdometryOldTimeSecs)) / (OdometryNewTimeSecs - OdometryOldTimeSecs);
	interpolated.wheelspeedleft = Odometryold.wheelspeedleft + ((Odometrynew.wheelspeedleft - Odometryold.wheelspeedleft) * (EKFfilterTimeSecs - OdometryOldTimeSecs)) / (OdometryNewTimeSecs - OdometryOldTimeSecs);

	std::cout << "Odometry old: Wheelspeed Left " << Odometryold.wheelspeedleft << " Wheelspeed Right " << Odometryold.wheelspeedright << "\n" << std::endl; //DB
	std::cout << "Odometry new: Wheelspeed Left " << Odometrynew.wheelspeedleft << " Wheelspeed Right " << Odometrynew.wheelspeedright << "\n" << std::endl; //DB
	std::cout << "Odometry intrepolated: Wheelspeed Left " << interpolated.wheelspeedleft << " Wheelspeed Right " << interpolated.wheelspeedright << "\n" << std::endl; //DB

	return interpolated;
}

/**
 * @brief Linear interpolation of values from measurement (motion capture)
 *
 * This function performs linear interpolation of robot pose2D for a given time instant.
 * The time for which the robot pose2D are computed is defined by EKFfilterTimeSecs.
 *
 * @param zOld contains robot pose2D and timestamp read at previous instant when EKF was updated.
 * @param zNew contains robot pose2D and timestamp read currently.
 * @param EKFfilterTimeSecs is the time instant for which the EKF update need to be performed and robot pose2D need to be computed.
 *
 * @return rollo::Pose2DStamped, contains robot pose2D computed for time instant given by EKFfilterTimeSecs using linear interpolation.
 */

//CRF
rollo::Pose2DStamped interpolateMeasurement( rollo::Pose2DStamped zOld, rollo::Pose2DStamped zNew, double EKFfilterTimeSecs ) {

	rollo::Pose2DStamped interpolated;

	double zOldTimeSecs = zOld.header.stamp.toSec();
	double zNewTimeSecs = zNew.header.stamp.toSec();

//CRC same as above
	//y = y0 + ((y1-y0)/(t1-t0))(t-t0)
	interpolated.pose2d.x = zOld.pose2d.x + ((zNew.pose2d.x - zOld.pose2d.x)*(EKFfilterTimeSecs - zOldTimeSecs))/(zNewTimeSecs - zOldTimeSecs);
	interpolated.pose2d.y = zOld.pose2d.y + ((zNew.pose2d.y - zOld.pose2d.y)*(EKFfilterTimeSecs - zOldTimeSecs))/(zNewTimeSecs - zOldTimeSecs);
	interpolated.pose2d.theta = zOld.pose2d.theta + ((zNew.pose2d.theta - zOld.pose2d.theta)*(EKFfilterTimeSecs - zOldTimeSecs))/(zNewTimeSecs - zOldTimeSecs);

//CRC Provide better description instead of old, new - previous, current, next
	std::cout << "Measurement old: [" << zOld.pose2d.x << "  " << zOld.pose2d.y << "  " << zOld.pose2d.theta  << "]^T\n" << std::endl;//DB
	std::cout << "Measurement new: [" << zNew.pose2d.x << "  " << zNew.pose2d.y << "  " << zNew.pose2d.theta  << "]^T\n" << std::endl;//DB
	std::cout << "Measurement intrepolated: [" << interpolated.pose2d.x << "  " << interpolated.pose2d.y << "  " << interpolated.pose2d.theta  << "]^T\n" << std::endl;//DB

	return interpolated;
}

/**
 * @brief FSTATE nonlinear state equations, f(x_k-1, u_k-1)
 *
 * This is part of time update(or prediction update) of EKF. Given "a priori" state estimate, x_k-1|k-1
 * and u_k-1, it computes predicted value for state, x_k|k-1.
 *
 * @param x_pp contains  "a priori" state estimate, x_k-1|k-1.
 * @param u is control input vector, calculated from odometry. It consists of 2 elements, delta S and delta theta.
 *
 * @return Eigen::Vector3d, state prediction x_k|k-1.
 */

Eigen::Vector3d FSTATE(Eigen::Vector3d x_pp, Eigen::Vector2d u) {

	Eigen::Vector3d x_cp(x_pp(0) + u(0) * cos(x_pp(2) - (u(1) / 2)), x_pp(1) + u(0) * sin(x_pp(2) - (u(1) / 2)), x_pp(2) - u(1));

	return x_cp;
}

/**
 * @brief JacobianFSTATE
 *
 * This computes Jacobian matrix by taking the partial derivatives of f(x_k-1,u_k-1) with respect to x.
 *
 * @param x_pp contains "a priori" state estimate, x_k-1|k-1.
 * @param u is control input vector, calculated from odometry. It consists of 2 elements, delta S and delta theta.
 *
 * @return Eigen::Matrix3d is the Jacobian matrix
 */

Eigen::Matrix3d JacobianFSTATE(Eigen::Vector3d x_pp, Eigen::Vector2d u) {

	Eigen::Matrix3d Jf_xu = Eigen::Matrix3d::Identity();
	Jf_xu(0, 2) = - u(0) * sin(x_pp(2) - (u(1) / 2));
	Jf_xu(1, 2) = u(0) * cos(x_pp(2) - (u(1) / 2));

	return Jf_xu;
}

/**
 * @brief HMEAS measurement equation, h(x_k)
 *
 * This computes estimated measurement vector based on the latest state estimate.
 *
 * @param x_cp contains state prediction x_k|k-1 computed in time update of EKF
 *
 * @return Eigen::Vector3d, contains estimated measurement vector.
 */

Eigen::Vector3d HMEAS(Eigen::Vector3d x_cp) {

	Eigen::Vector3d h_x (x_cp(0), x_cp(1), x_cp(2));

	return h_x;
}


/**
 * @brief Node main
 *
 * Initialize node, nodehandle, subscribe to messages from preprocessor and communication nodes and publish estimated state of the robot.
 *
 * @param rate: Sampling frequency of the node <!1 [Hz]>
 *
 * Initializes Extended Kalman Filter revelant variables.
 * As a part of initializing, function waits for one message from each subscriber and save timestamps for the first iteration of EKF.
 * State estimate, x_(0|-1) is initialized as the first measurement read from the preprocessor node.
 * Covariance of state estimate, E(0, -1) is initialized as identity matrix.
 *
 * Run EKF in loop, update estimates.
 * Await new sensor data, determine time step for EKF update and perform necessary interpolation.
 *
 * Publishes newest estimates of state variables, covariance matrix and timestamp.
 * @return 0
 */

int main(int argc, char **argv)
{
//! ## Initialize
//! - Initialize node
ros::init(argc, argv, "rollo_ekf"); // Name of the node
ros::start(); // Necessary to be called

//! - Initialize nodehandle for subscribers and publisher
ros::NodeHandle RolloEKF;

//! - Initialize subscribers:
//!   - Initialize subscriber for measurement data
//!   - Initialize subscriber for actual speed of wheels, preferably extracted from encoders or if not available by using a lookup table
ros::Subscriber MeasSub = RolloEKF.subscribe(TopicPose2DStamped, 1024, subscriberCallbackMeasurement);
ros::Subscriber ContSub = RolloEKF.subscribe(TopicWheelSpeed, 1024, subscriberCallbackControlInput);

//! - Initialize publisher and define topic and message queue size for the publisher
ros::Publisher RolloPub = RolloEKF.advertise<rollo::EKF>(TopicEKF, 1024);

//! - Initialize node arguments using command line
int rate_frequency;

//! - Initialize node parameters from launch file or command line
//! Use a private node handle so that multiple instances of the node can be run simultaneously
//! while using different parameters.
ros::NodeHandle private_node_handle_("~");
private_node_handle_.param("rate", rate_frequency, int(1));

//! - Publishing rate [Hz]
ros::Rate frequency(rate_frequency);

//! - Loop condition variable
int loopcondition = 1;

//! - Initialize variables involved in computation of EKF:
//!   - Define number of states
//!   - Initialize noise covariances and matrices
int nstates = 3; // Number of states: 3 = (position) (x, y) (orientation) (Theta)

double q = 0.1; // std of process noise   //CRC
double r = 0.1; // std of measurement noise //CRC

//!   - Process noise covariance
Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();

Q(0,0) = q*q;
Q(1,1) = q*q;
Q(2,2) = q*q;

//!   - Measurement noise covariance
Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

R(0,0) = r*r;
R(1,1) = r*r;
R(2,2) = r*r;

std::cout << "Q:\n" << Q << "\nR:\n" << R << "\n" << std::endl; //VB

//!   - Initialize vector for control input u and variables involved in its computation
Eigen::Vector2d u(0, 0);

double SL = 0;
double SR = 0;
double dt = 1; // Time from timestamp
double nL = 1;
double nR = 1;
double deltaTheta;
double deltaS;

//!   - Initialize state estimate vector "a priori" and measurement vector
Eigen::Vector3d x_pp(0, 0, 0);
//Eigen::Vector3d z(0, 0, 0);

//!   - Initialize Jacobian matrix with the partial derivatives of h(x_k) with respect to x, identity for provided system
Eigen::Matrix3d Jh = Eigen::Matrix3d::Identity();

//!   - Initialize E_pp: "a priori" estimated state covariance, E_k-1|k-1 (p refers to k-1)
Eigen::Matrix3d E_pp = Eigen::Matrix3d::Identity(); // Initial state covariance matrix

//!   - Initialize variables involved in the prediction update of EKF
Eigen::Vector3d x_cp; // State prediction, x_k|k-1
Eigen::Matrix3d Jf;
Eigen::Matrix3d E_cp; // E_k|k-1

//!   - Initialize variables involved in the innovation update of EKF
Eigen::Vector3d z_estimate;
Eigen::Matrix3d P12;
Eigen::Matrix3d S_inv;
Eigen::Matrix3d H;
Eigen::Matrix3d L;

//!   - Initialize state estimate vector and state covariance matrix a posteriori
Eigen::Vector3d x_cc;
Eigen::Matrix3d E_cc;

//! - Initialize variables involved in Odometry update alone
Eigen::Vector3d x_pp_odom(0, 0, 0);
Eigen::Vector3d x_cc_odom(0, 0, 0);

//! - Variables for time
char flagSensorsDataAvailable = 0;
double prevOdometrySecs = 0;
double prevMeasurementSecs = 0;

rollo::WheelSpeed prevOdometry;
rollo::WheelSpeed InterpolatedOdometry;
rollo::Pose2DStamped prevzPose2DStamped;
rollo::Pose2DStamped InterpolatedMeasurement;

double EKFfilterTimeSecs;
double PreviousEKFfilterTimeSecs;
double wheelspeedleft_EKF;
double wheelspeedright_EKF;
Eigen::Vector3d z_EKF(0, 0, 0);

char initialize = 1;

//! - Initialize measurement vector with timestamp and odometry data with timestamp from subscriber messages
//! - Initialize state estimate using measurement vector reading

std::cout << "Initializing: waiting for sensor data from both the subscribers \n" << std::endl; //DB

//! ## Initialization loop
while (initialize == 1 && ros::ok()) {

	//! - Check if data is available from measurement (motion capture)
	if (zTimeSecs > 0 ){
		//!  - Initialize @p prevzPose2DStamped, @p prevMeasurementSecs and @p PreviousEKFfilterTimeSecs
		prevzPose2DStamped = zPose2DStamped;
		prevMeasurementSecs = zTimeSecs;
		PreviousEKFfilterTimeSecs = zTimeSecs;
	}

	//! - Check if data is available from odometry (control input)
	if (OdometryTimeSecs > 0 ){
		//!  - Initialize @p prevOdometry, @p prevOdometrySecs and @p PreviousEKFfilterTimeSecs
		prevOdometry = Odometry;
		prevOdometrySecs = OdometryTimeSecs;
		PreviousEKFfilterTimeSecs = OdometryTimeSecs;
	}

	//! - Check if new data has been read from  both measurement (motion capture) and odometry (control input)
	if (prevMeasurementSecs > 0 && prevOdometrySecs > 0){
		//!  - Initialize initial state estimate @p x_pp
		initialize = 0;
		x_pp(0) = zPose2DStamped.pose2d.x;
		x_pp(1) = zPose2DStamped.pose2d.y;
		x_pp(2) = zPose2DStamped.pose2d.theta;
		x_pp_odom = x_pp;

		//!  - Initialization done
	std::cout << " Initialization done:  \nx_(0|-1):\n" << x_pp << "\nE_pp_(0|-1):\n" << E_pp << "\n" << std::endl;//DB
	std::cout << " Initial time step t0:\n" << PreviousEKFfilterTimeSecs << "\n" << std::endl;//DB

	}

	ros::spinOnce();
	frequency.sleep();

}
//! ## Initialization loop end


//! ## Main loop
do {

	std::cout << "Wait for new data from all sensors (motion captutre and odometry) for next EKF iteration. \n" << std::endl; //DB

	//! - Check if new data is available from measurement (motion capture) and odometry (control input)
	if (OdometryTimeSecs > prevOdometrySecs && zTimeSecs > prevMeasurementSecs) {
		flagSensorsDataAvailable = 1;
		prevOdometrySecs = OdometryTimeSecs;
		prevMeasurementSecs = zTimeSecs;

		//!   - Determine time step for EKF update and perform interpolation for the sensor data not available at respective time step
		if (zTimeSecs > OdometryTimeSecs) {
			//! - Update timestamp
			EKFfilterTimeSecs = OdometryTimeSecs;
			//! - Interpolate measurements
			InterpolatedMeasurement = interpolateMeasurement(prevzPose2DStamped, zPose2DStamped, EKFfilterTimeSecs);
			//! - Update state
			z_EKF(0) = InterpolatedMeasurement.pose2d.x;
			z_EKF(1) = InterpolatedMeasurement.pose2d.y;
			z_EKF(2) = InterpolatedMeasurement.pose2d.theta;
			wheelspeedleft_EKF = Odometry.wheelspeedleft;
			wheelspeedright_EKF = Odometry.wheelspeedright;

		std::cout << "Timestamp for next iteration: "<< EKFfilterTimeSecs << "\n" << std::endl;//DB
		std::cout << "Linear interpolation of measurement required for EKF iteration." << "\n" << std::endl;//DB
		std::cout << "Interpolated measurement vector: \n" << z_EKF <<"\n" << std::endl;//DB
		std::cout << "Odometry: Wheelspeed: L: " << wheelspeedleft_EKF << " R: " << wheelspeedright_EKF << "\n" << std::endl;//DB

		} else {

			//! - Update timestamp
			EKFfilterTimeSecs = zTimeSecs;

			//! - Interpolate for measurement
			InterpolatedOdometry = interpolateOdometry(prevOdometry, Odometry, EKFfilterTimeSecs);

			//! - Update variables involved in EKF update
			z_EKF(0) = zPose2DStamped.pose2d.x;
			z_EKF(1) = zPose2DStamped.pose2d.y;
			z_EKF(2) = zPose2DStamped.pose2d.theta;
			wheelspeedleft_EKF = InterpolatedOdometry.wheelspeedleft;
			wheelspeedright_EKF = InterpolatedOdometry.wheelspeedright;

			std::cout << "Time stamp for next iteration: "<< EKFfilterTimeSecs << "\n" << std::endl;//DB
			std::cout << "Linear Interpolation of odometry required for EKF iteration." << "\n" << std::endl;//DB
			std::cout << "Interpolated Odometry: Wheelspeed Left " << wheelspeedleft_EKF << " Wheelspeed Right " << wheelspeedright_EKF << "\n" << std::endl;//DB
			std::cout << "Measurement vector: \n" << z_EKF <<"\n" << std::endl;//DB

		}

	//! - Update @p prevOdometry and @p prevzPose2DStamped for next loop
	prevOdometry = Odometry;
	prevzPose2DStamped = zPose2DStamped;

	}

	//! ### EKF update
	//! Perform EKF update if all sensor data is available:
	if (flagSensorsDataAvailable == 1) {

		flagSensorsDataAvailable = 0;
		std::cout << "Perform EKF iteration. \n" << std::endl; //DB

		//CRD
		//! - Determine dt
		dt = EKFfilterTimeSecs - PreviousEKFfilterTimeSecs;

		//! - Update PreviousEKFfilterTimeSecs for the next loop
		PreviousEKFfilterTimeSecs = EKFfilterTimeSecs;

//CRC cleanup
		//! - Determine control input u from nL and nR
		//nL = wheelspeedleft_EKF * 60 / (2 * PI);
		//nR = wheelspeedright_EKF * 60 / (2 * PI);
		SL = dt * wheelspeedleft_EKF * ROLLO_WHEEL_RADIUS_L; // Linear distance traveled by left wheel in meters
		SR = dt * wheelspeedright_EKF * ROLLO_WHEEL_RADIUS_R; // Linear distance traveled by right wheel in meters
		//SL = dt * (nL / 60) * 2 * PI * ROLLO_WHEEL_RADIUS_L; // Linear distance traveled by left wheel in meters
		//SR = dt * (nR / 60) * 2 * PI * ROLLO_WHEEL_RADIUS_R; // Linear distance traveled by right wheel in meters
		deltaTheta = (SL - SR) / ROLLO_AXLE_L; //%u(2)
		deltaS = (SL + SR) / 2; //% u(1)
		u << deltaS, deltaTheta;

//CRC
		//  std::cout << "nL: " << nL << " nR: " << nR << "\n" << std::endl;//DB
		std::cout << "dt: " << dt << "\n" << std::endl;//DB
		std::cout << "SL: " << SL << " SR: " << SR << "\n" << std::endl;//DB
		std::cout << "deltaTheta " << deltaTheta << " deltaS " << deltaS << "\n" << std::endl;//DB
		std::cout << "u:\n" << u << "\n" << std::endl;//DB

		//! - Prediction update
		//! - Nonlinear update and linearization at current state
		x_cp = FSTATE(x_pp, u); //f_xu; //%state prediction, x_k|k-1
		Jf = JacobianFSTATE(x_pp, u);

		std::cout << "x_cp:\n" << x_cp << "\nE_pp:\n" << E_pp << "\n" << std::endl; //DB
		std::cout << "Jf_xu:\n" << Jf << "\nJh:\n" << Jh << "\n" << std::endl; //DB

		//! - Partial covariance update
		//CRC
		E_cp = Jf * E_pp * Jf.transpose() + Q;  // %E_k|k-1
		std::cout << " Prediction Update: \nx_cp:\n" << x_cp << "\nE_cp:\n" << E_cp << "\n" << std::endl; //DB

		//! - Innovation update
		//! - Nonlinear measurement and linearization
		z_estimate = HMEAS(x_cp);
		std::cout << " Measurement estimate:\n" << z_estimate << "\n" << std::endl; //DB

		P12 = E_cp * Jh.transpose(); //! Cross covariance

/*
		S_inv = (Jh * P12 + R).inverse();
		H = P12*S_inv; //     %Kalman filter gain, H_k
		std::cout << " Kalman filter gain: \n" << H << "\n" << std::endl; //DB

		//CRC
		x_cc = x_cp + H * (z_EKF - z_estimate); //    %state estimate, x_k|k;
		E_cc = E_cp - H * P12.transpose();  //             %state covariance matrix, E_k|k
// */

		//! - Cholesky decomposition:
		//! Instead of standard equations for EKF, use cholesky factorization for a stable covariance matrix
		//!   - Compute the Cholesky decomposition
		Eigen::LLT<Eigen::Matrix3d, Eigen::Upper> chol(Jh * P12 + R);
		L = chol.matrixU();

		R = L;
		H = P12 * R.inverse();

		std::cout << " Verify Cholesky: \nS:\n" << (Jh * P12 + R) << "\nL:\n" << L << "\n Verify\n" << L.transpose() * L << std::endl; //DB
		std::cout << " P12:\n" << P12 << "\n R:\n" << R << "\nRinv:\n" << R.inverse() << "\n" <<std::endl; //DB
		std::cout << " H:\n" << H << "\n" <<std::endl; //DB

		//! - Compute state estimate x_k|k and state covariance matrix E_k|k
		x_cc = x_cp + H * ((R.transpose() ).inverse()) * (z_EKF - z_estimate);  //! State estimate x_k|k
		E_cc = E_cp - H * H.transpose(); //! State covariance matrix E_k|k

		//! - Update previous covariance matrix E_pp an state x_pp values for next loop
		x_pp = x_cc;
		E_pp = E_cc;
		std::cout << " Final Measurement Update: \nx_cc:\n" << x_cc << "\nE_cc:\n" << E_cc << "\n" << std::endl; //DB

		//! - Determine odometry update without filter
		x_cc_odom = FSTATE(x_pp_odom, u); //f_xu
		x_pp_odom = x_cc_odom;

		//! - Reinitialize R for the next loop
		R = Eigen::Matrix3d::Identity();

		R(0, 0) = r * r;
		R(1, 1) = r * r;
		R(2, 2) = r * r;

		//! - Prepare data for publishing
		rollo::EKF result;
		result.header.stamp.sec = (int32_t) EKFfilterTimeSecs;
		result.header.stamp.nsec = (int32_t)((EKFfilterTimeSecs - result.header.stamp.sec) * 1000000000); // 1.000.000.000 //Q what is the limit of int here? it is possible to overflow? If so, when? Does that make sense?

		//! - Pose2D EKF
		result.ekfpose2d.x = x_cc(0);
		result.ekfpose2d.y = x_cc(1);
		result.ekfpose2d.theta = x_cc(2);

		//! - Covariance
		result.covariance[0] = E_cc(0,0);
		result.covariance[1] = E_cc(0,1);
		result.covariance[2] = E_cc(0,2);
		result.covariance[3] = E_cc(1,0);
		result.covariance[4] = E_cc(1,1);
		result.covariance[5] = E_cc(1,2);
		result.covariance[6] = E_cc(2,0);
		result.covariance[7] = E_cc(2,1);
		result.covariance[8] = E_cc(2,2);

		//! - Pose2D odometry
		result.odompose2d.x = x_pp_odom(0);
		result.odompose2d.y = x_pp_odom(1);
		result.odompose2d.theta = x_pp_odom(2);


		//! - Publish
		RolloPub.publish(result);

	}
	//! ### EKF Update end

	ros::spinOnce();

	// ROS_INFO("[Rollo][Debug][Counter]: %d", loopcounter); //DB

	if (! ros::ok()) loopcondition = 0;

	//! - Synchronize to rate
	frequency.sleep();

} while (loopcondition);
//! ## Main loop end

return 0;
}
