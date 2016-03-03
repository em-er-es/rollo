#!/bin/bash
DIRG="Graphics";
EXT1="svg";
EXT2="pdf";
DPI=96;
TOL=10;

for FILE in "$DIRG/"*".$EXT1"; do
	FILE2="$(echo "$FILE" | sed "s/$EXT1\$/$EXT2/g")";
	MODTIME1=$(stat -c %Y "$FILE");
	MODTIME2=$(stat -c %Y "$FILE2");
	MODDIF=$((MODTIME1 - MODTIME2));
	if [[ $MODDIF -gt $TOL ]]; then
		inkscape -z -D --file="$FILE" --export-pdf="$FILE2" --export-dpi=$DPI --export-latex
	fi
done
