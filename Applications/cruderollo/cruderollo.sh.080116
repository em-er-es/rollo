#!/bin/bash
if [[ -z $1 ]]; then ROLLO_IP=192.168.0.120; else ROLLO_IP=$1; fi
if [[ -z $2 ]]; then ROLLO_UDP_PORT=900; else ROLLO_UDP_PORT=$2; fi
if [[ -z $3 ]]; then ROLLO_REPEAT_CMD=3; else ROLLO_REPEAT_CMD=$3; fi

connect_rollo(){
echo "Connecting to $ROLLO_IP@$ROLLO_UDP_PORT";
if [[ $VERBOSE ]]; then
	echo "echo 0x000000 | netcat -u $ROLLO_IP $ROLLO_UDP_PORT & (sleep 3 && pkill netcat)";
fi
echo 0x000000 | netcat -u -v $ROLLO_IP $ROLLO_UDP_PORT & (sleep 3 && pkill netcat);
}

udp_rollo(){
echo sendip -p ipv4 -p udp -ud $ROLLO_UDP_PORT -d $@ -v $ROLLO_IP;
#sendip -p ipv4 -p udp -ud $ROLLO_UDP_PORT -d $@ -v $ROLLO_IP;
sudo sendip -p ipv4 -p udp -ud $ROLLO_UDP_PORT -d $@ -v $ROLLO_IP;
}

exit_rollo(){
echo -e "\nExiting...\nStopping Rollo";
for i in $(seq 10); do udp_rollo 0x7b5031 &> /dev/null; done
exit 0;
}

help_rollo(){
echo -e "\nRollo crude control script\nCommands:\n";
echo -e "MOV:[fF]orward\t[bB]ackward\t[lL]eft\t[rR]ight\n[sS]top\t\tre[C]onnect\t[Q]uit\n";
echo -e "VEL:[0]* = 0%\t[1]* = 12%\t[2]* = 25%\t[3]* = 37%\n[4]* = 50%\t[5]* = 62%\t[6]* = 75%\t[7]* = 87%\n\t\t\t\t\t\t[8|9]* = 100%\n";
#echo -e "\n";
}

help_rollo;
connect_rollo;
#echo -e "Ready."
echo -e "Input:"
MSG3=31;
STOP=0;
TMP=$ROLLO_REPEAT_CMD;
COUNTER=1;
while :; do
#	read CMD
	echo -e "\nMOV:"
	read -n1 CMD1
#	MOV=${CMD[1]}; VEL=${CMD[2]};
	MOV=$CMD1;
#	echo $MOV $VEL
#	echo $CMD1 $CMD2
	case "$MOV" in
		[fF]*)
		MSG1=7c;
		INFO1="FORWARD";
		;;
		[bB]*)
		MSG1=7d;
		INFO1="BACKWARD";
		;;
		[lL]*)
		MSG1=7e;
		INFO1="LEFT";
		;;
		[rR]*)
		MSG1=7f;
		INFO1="RIGHT";
		;;
		[sS]*)
		MSG1=7b;
		MSG2=50;
		INFO1="STOP";
		INFO2="0%";
		STOP=1;
		;;
#		[qQeExX]*)
		Q*)
#		echo -e "\nExiting."
		exit_rollo;
		;;
		C*)
		echo -e "\nReconnecting."
		connect_rollo;
		continue;
		;;
		[hH]*)
#		echo -e "\nPrint help."
		help_rollo;
		;;
		*)
		echo -e "\nParamter 1 not understood. Repeat"
#		echo -e "\nParamter 1 not understood. Exiting"
#		exit 1;
		continue;
		;;
	esac
	if [[ $STOP == 0 ]]; then
	echo -e "\nVEL:";
	read -n1 CMD2
	VEL=$CMD2;
	case "$VEL" in
		0*)
		MSG2=50;
		INFO2="0%";
		;;
		1*)
		MSG2=55;
		INFO2="12%";
		;;
		2*)
		MSG2=56;
		INFO2="25%";
		;;
		3*)
		MSG2=57;
		INFO2="37%";
		;;
		[45]*)
		MSG2=59;
		INFO2="50%";
		;;
		6*)
		MSG2=5F;
		INFO2="62%";
		;;
		7*)
		MSG2=60;
		INFO2="75%";
		;;
		8*)
		MSG2=61;
		INFO2="87%";
		;;
		9*)
		MSG2=62;
		INFO2="100%";
		;;
#		10*)
#		MSG2=62;
#		INFO2="100%";
#		;;
#		[qQeExX]*)
		[sS]*)
		MSG1=7b;
		MSG2=50;
		INFO1="STOP";
		INFO2="0%";
		STOP=1;
		;;
		[Q]*)
#		echo -e "\nExiting."
		exit_rollo;
		;;
		C*)
		echo -e "\nReconnecting."
		connect_rollo;
		continue;
		;;
		[hH]*)
#		echo -e "\nPrint help."
		help_rollo;
		;;
		*)
		echo -e "\nParamter 2 not understood. Repeat"
#		echo -e "\nParamter 2 not understood. Exiting"
#		exit 1;
		continue;
		;;
	esac
	fi
	echo -e "\nRollo: $INFO1 @ $INFO2 speed";
	if [[ $STOP ]] && [[ $ROLLO_REPEAT_CMD < 6 ]]; then TMP=$ROLLO_REPEAT_CMD; ROLLO_REPEAT_CMD=6; fi
	if [[ $VERBOSE ]]; then
	for i in $(seq $ROLLO_REPEAT_CMD); do udp_rollo 0x$MSG1$MSG2$MSG3; done
	else
	for i in $(seq $ROLLO_REPEAT_CMD); do udp_rollo 0x$MSG1$MSG2$MSG3 &> /dev/null; done
	fi
	unset MSG2;
	if [[ $STOP ]]; then ROLLO_REPEAT_CMD=$TMP; fi
	STOP=0;
	echo Counter: $COUNTER;
	COUNTER=$((COUNTER + 1));
done
exit_rollo;
