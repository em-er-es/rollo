# University of Genova
# 86805 - Software Architectures for Robotics
# Assignment 03 - Localization system for a wheeled humanoid robot
# Rabbia Asghar, Ernest Skrzypczyk

### README

Project: Rollo - Localization of a humanoid robot

Summary: Project deals with the aspect of localization of a humanoid robot using the extended Kalman filter and ROS as working environment with help of motion capture.

Description: Given was the humanoid robot Rollo with a mechanical and electrical construction that enabled reesembling human walking motion.

Goal: Implementation of localization of Rollo using ROS environment and extended Kalman filter with motion capture data as one of the information sources.

Tasks performed for the localization of Rollo using a ROS environment with the help of a motion capture system included:
* Modelling of the physical system of the robot
* Taking into consideration odometry errors: systematic and non-systematic
* Performing the square test based on UMBMark
* Designing the system and measurement models
* Implementing the extended Kalman filter into ROS
* Developing all necessary nodes to control and communicate with Rollo using ROS
* Developing nodes for visualization and preprocessing of motion capture data

The localization has been successfully implemented. There is still room for improvement, especially with the odometry model, but that requires more intesive testing on the robot. Furthermore the ROS nodes could be improved:
* Visualization node could also provide analysis of the EKF performance
* Visualization node could be greatly expanded for media purposes by fully utilizing already provided code like saving animations and screenshots
* EKF node could reinitialize the odometry model at a given refresh period, should the model be incomplete
* Implementation of dynamic reconfiguration server into the ROS nodes, would greatly improve work with prototypes and changing environments

Further changes on the Rollo should include:
* Use of better connectors in the power circuits is necessary, even though wire connections have been greatly improved already
* Provide at least a basic shortage protection circuit: This can be done using diodes, like 1N4007, on the input and output of the voltage regulators; One in series to rectify and one in parallel to protect in case of opposite polarities
* Exchange and calibration of wheels is necessary
* A different material for the wheel surface might be an asset

GitHub repository: Serves for the purpose of sharing the developed code with other and allow further improvements.

Files and directories of interest:
Localization system for a wheeled humanoid robot.pdf --  Main document
Documentation/Localization system for a wheeled humanoid robot -- ROS documentation.pdf -- LaTeX version
Documentation/Localization system for a wheeled humanoid robot -- ROS documentation.html -- HTML version
Documentation/Localization system for a wheeled humanoid robot -- ROS documentation.pdf.7z -- Compressed archive of LaTeX version
Documentation/Localization system for a wheeled humanoid robot -- ROS documentation.html.7z -- Compressed archive of HTML version
Graphics/Images/ -- Project documentation
Videos/ -- Videos of Rollo under operation in the motion capture environment and the developed software in action
rollo/ -- ROS workspace tree - Includes source code for nodes and configuration files
ROS/ -- ROS specific files for launchers and packages in general
Scripts/ -- Various scripts developed to aid the progess in project
Logs/ -- Various logs in different formats with corresponding scripts for offline analysis
