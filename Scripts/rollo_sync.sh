#!/bin/bash
PACKAGE="rollo";
ROS_DIR1="$HOME/.ros/workspace/devel/lib/$PACKAGE";
ROS_DIR2="$HOME/.ros/workspace/src/$PACKAGE/bin";


echo rsync -rv "$ROS_DIR1/" "$ROS_DIR2/";
rsync -rv "$ROS_DIR1/" "$ROS_DIR2/";
