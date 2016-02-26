#!/bin/bash
PACKAGE="rollo";
ROS_DIR0="$HOME/.ros/workspace/build/$PACKAGE";
ROS_DIR1="$HOME/.ros/workspace/devel/lib/$PACKAGE";
ROS_DIR2="$HOME/.ros/workspace/src/$PACKAGE/bin";
ROS_DIR3="$HOME/.ros/workspace/devel/share/$PACKAGE/cmake";
ROS_DIR4="$HOME/.ros/workspace/devel/include/$PACKAGE";


echo rm -Iv "$ROS_DIR0/*";
rm -Iv "$ROS_DIR0"/*;
echo rm -Iv "$ROS_DIR1/*";
rm -Iv "$ROS_DIR1"/*;
echo rm -Iv "$ROS_DIR2/*";
rm -Iv "$ROS_DIR2"/*;
echo rm -Iv "$ROS_DIR3/*";
rm -Iv "$ROS_DIR3"/*;
echo rm -Iv "$ROS_DIR4/*";
rm -Iv "$ROS_DIR4"/*;
