#!/bin/bash
echo -e "ROS publisher helper\n"
echo -e "FILE [MESSAGE_DESTINATION MESSAGE_FORMAT LOOPS FREQUENCY PERIOD PRESLEEP]\n"
#echo -e "FILE [MESSAGE_DESTINATION MESSAGE_FORMAT LOOPS FREQUENCY]\n"
#echo -e "FILE [MESSAGE_DESTINATION MESSAGE_FORMAT LOOPS PERIOD]\n"

FILE="$1";
if [[ $# -lt 1 ]]; then echo "Specify log file to publish"; exit 1; fi
if [[ ! -f "$FILE" ]]; then echo -e "Error reading $FILE"; exit 1; fi

if [[ $# -lt 2 ]]; then 
	MESSAGE_DESTINATION="/Optitrack_Rollo/ground_pose"; MESSAGE_FORMAT="geometry_msgs/Pose2D"; LOOPS=0; FREQUENCY=25.0; PERIOD=0; PRESLEEP=0;
else
	MESSAGE_DESTINATION=$2; MESSAGE_FORMAT=$3; LOOPS=$4; FREQUENCY=$5; PERIOD=$6; PRESLEEP=$7;
fi

if [[ $LOOPS -le 0 ]]; then 
	LOOPS=1; DEC=0;
else
	DEC=1;
fi

if [[ -z $ROS_PACKAGE_PATH ]]; then
	source /opt/ros/jade/setup.zsh
	export PYTHONPATH=/opt/ros/jade/lib/python2.7/site-packages:$PYTHONPATH 
	export PKG_CONFIG_PATH="/opt/ros/jade/lib/pkgconfig:$PKG_CONFIG_PATH" 
	export ROS_PACKAGE_PATH="$HOME/.ros/workspace/devel:$HOME/.ros/workspace:$ROS_PACKAGE_PATH" 
	alias catkin_make="catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python2 -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/libpython2.7.so"
fi

#while [[ $LOOPS -gt 0 ]]; do
#	while read LINE; do
#		rostopic pub -1 "$MESSAGE_DESTINATION" "$MESSAGE_FORMAT" -- "$VAR1" "$VAR2" "$VAR3";
#echo		rostopic pub -1 "$MESSAGE_DESTINATION" "$MESSAGE_FORMAT" -- $LINE;
#		rostopic pub -1 "$MESSAGE_DESTINATION" "$MESSAGE_FORMAT" -- $LINE;
#		sleep $((1/FREQUENCY));
#	done < "$FILE";
#	LOOPS=$((LOOPS - DEC));
#	sleep $((PERIOD));
#done

sleep $PRESLEEP;
while [[ $LOOPS -gt 0 ]]; do
echo	rostopic pub "$MESSAGE_DESTINATION" "$MESSAGE_FORMAT" -r $FREQUENCY -f "$FILE";
	rostopic pub "$MESSAGE_DESTINATION" "$MESSAGE_FORMAT" -r $FREQUENCY -f "$FILE";
	LOOPS=$((LOOPS - DEC));
	sleep $PERIOD;
done
