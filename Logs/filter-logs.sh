#!/bin/bash
DIR="Filtered";
if [[ ! -d "$DIR" ]]; then mkdir -v "$PWD/$DIR"; fi
for F in *.log ; do
	awk '/Sub/{x = $NF - 2; y = $NF - 1; theta = $NF; print "x: " x "\ny: " y "\ntheta: " theta "\n---"}' $F | tr -cd '\11\12\15\40-\176' | sed 's/\[0m//g; $d' > $DIR/$F;
done
