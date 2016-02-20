#!/bin/bash
## Arguments
if [[ -z $1 ]]; then ROLLO_IP=192.168.0.120; else ROLLO_IP=$1; fi
if [[ -z $2 ]]; then ROLLO_UDP_PORT=900; else ROLLO_UDP_PORT=$2; fi
if [[ -z $3 ]]; then ROLLO_REPEAT_CMD=3; else ROLLO_REPEAT_CMD=$3; fi

## Definitions
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
echo -e "MOV:[fF]orward\t[bB]ackward\t[lL]eft\t[rR]ight\n[sS]top\t\tre[C]onnect\t[Q]uit";
echo -e "[fblr] = Independent wheel speed\t[FBLR] = Same speed\n"
echo -e "VEL:[0]* = 6%\t[1]* = 12%\t[2]* = 19%\t[3]* = 25%\n[4]* = 31%\t[5]* = 38%\t[6]* = 44%\t[7]* = 50%\n\t\t\t\t\t\t[8|9]* = 56%\n";
#echo -e "\n";
}

## Initialization
help_rollo;
#connect_rollo;
#echo -e "Ready."
echo -e "Input:"
#MSG3=31;
STOP=0;
TMP=$ROLLO_REPEAT_CMD;
COUNTER=1;
## Main loop
while :; do
## Movement
	echo -e "\nMOV:"
	read -n1 CMD1
	MOV=$CMD1;
	case "$MOV" in
		[f]*)
		MSG1=7c;
		INFO1="FORWARD";
		INDEPENDENT=1;
		;;
		[b]*)
		MSG1=7d;
		INFO1="BACKWARD";
		INDEPENDENT=1;
		;;
		[l]*)
		MSG1=7e;
		INFO1="LEFT";
		INDEPENDENT=1;
		;;
		[r]*)
		MSG1=7f;
		INFO1="RIGHT";
		INDEPENDENT=1;
		;;
		[F]*)
		MSG1=7c;
		INFO1="FORWARD";
		INDEPENDENT=0;
		;;
		[B]*)
		MSG1=7d;
		INFO1="BACKWARD";
		INDEPENDENT=0;
		;;
		[L]*)
		MSG1=7e;
		INFO1="LEFT";
		INDEPENDENT=0;
		;;
		[R]*)
		MSG1=7f;
		INFO1="RIGHT";
		INDEPENDENT=0;
		;;
		[sS]*)
		MSG1=7b;
		MSG2=50;
		INFO1="STOP";
		INFO2="$MSG2%";
		INFO3="$MSG3%";
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
## Speed left wheel
	if [[ $STOP == 0 ]]; then
	if [[ $INDEPENDENT ]]; then WHEEL="LEFT"; else WHEEL="BOTH"; fi
	echo -e "\n$WHEEL VEL:";
	read -n1 VELL
	case "$VELL" in
		0*)
		MSG2=50;
		INFO2="6%";
		;;
		1*)
		MSG2=55;
		INFO2="12%";
		;;
		2*)
		MSG2=56;
		INFO2="19%";
		;;
		3*)
		MSG2=57;
		INFO2="25%";
		;;
		4*)
		MSG2=59;
		INFO2="31%";
		;;
		5*)
		MSG2=5F;
		INFO2="38%";
		;;
		6*)
		MSG2=60;
		INFO2="44%";
		;;
		7*)
		MSG2=61;
		INFO2="50%";
		;;
		[89]*)
		MSG2=62;
		INFO2="56%";
		;;
#		10*)
#		MSG2=62;
#		INFO2="100%";
#		;;
#		[qQeExX]*)
		[sS]*)
		MSG1=7b;
		INFO1="STOP";
		INFO2="$MSG2%";
		INFO3="$MSG3%";
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
## Speed right wheel
	if [[ $STOP == 0 ]]; then 
	if [[ $INDEPENDENT == 0 ]]; then 
		VELR=$VELL; 
	else 
		WHEEL="RIGHT";
		echo -e "\n$WHEEL VEL:";
		read -n1 VELR
	fi
	case "$VELR" in
		0*)
		MSG3=10;
		INFO3="6%";
		;;
		1*)
		MSG3=11;
		INFO3="12%";
		;;
		2*)
		# MSG3=12; # Causes erratic behaviour
		MSG3=25; # Works as a workaround
		INFO3="19%";
		;;
		3*)
		MSG3=13;
		INFO3="25%";
		;;
		4*)
		MSG3=1A;
		INFO3="31%";
		;;
		5*)
		MSG3=1B;
		INFO3="38%";
		;;
		6*)
		MSG3=1C;
		INFO3="44%";
		;;
		7*)
		MSG3=1D;
		INFO3="50%";
		;;
		[89]*)
		MSG3=24;
		INFO3="56%";
		;;
#		10*)
#		MSG3=62;
#		INFO3="100%";
#		;;
#		[qQeExX]*)
		[sS]*)
		MSG1=7b;
		INFO1="STOP";
		INFO2="$MSG2%";
		INFO3="$MSG3%";
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
		echo -e "\nParamter 3 not understood. Repeat"
#		echo -e "\nParamter 3 not understood. Exiting"
#		exit 1;
		continue;
		;;
	esac
	fi
	fi
## Data transmission
	echo -e "\nRollo: $INFO1 @ L $INFO2 R $INFO3 speed";
	if [[ $STOP ]] && [[ $ROLLO_REPEAT_CMD < 6 ]]; then TMP=$ROLLO_REPEAT_CMD; ROLLO_REPEAT_CMD=6; fi
	if [[ $VERBOSE ]]; then
	for i in $(seq $ROLLO_REPEAT_CMD); do udp_rollo 0x$MSG1$MSG2$MSG3; done
	else
	for i in $(seq $ROLLO_REPEAT_CMD); do udp_rollo 0x$MSG1$MSG2$MSG3 &> /dev/null; done
	fi
	unset MSG2;
	unset MSG3;
	if [[ $STOP ]]; then ROLLO_REPEAT_CMD=$TMP; fi
	STOP=0;
	echo Counter: $COUNTER;
	COUNTER=$((COUNTER + 1));
done
exit_rollo;
