#!/bin/bash
DIR="Filtered";
if [[ ! -d "$DIR" ]]; then mkdir -v "$PWD/$DIR"; fi
for F in *.log ; do
	awk '/Sub/{t = $3; x = $NF - 2; y = $NF - 1; theta = $NF;  print "t: " t "\nx: " x "\ny: " y "\ntheta: " theta "\n---"}' $F | tr -cd '\11\12\15\40-\176' | sed -e 's/\[//g' -e 's/\]\://g' -e 's/\[0m//g; $d' > $DIR/$F;
	LINES=$(wc -l "$DIR/$F" | awk '{print $1}');
	sed -i -n 1,"$((LINES - 2))"p "$DIR/$F";
done
