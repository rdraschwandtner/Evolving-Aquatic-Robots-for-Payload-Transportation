#!/bin/bash

# Script assumes that it is placed in the same folder as MyFirstEvolveableWaterSnake.py

#absolute path to file, or relative to the script
IN_FILE="$1"
OUT_PATH="$2"

echo "IN_FILE: " $IN_FILE
echo "OUT_PATH: " $OUT_PATH

if [ -z "$IN_FILE" ]; then echo "IN_FILE is unset or set to the empty string, exit script"; exit; fi
if [ -z "$OUT_PATH" ]; then echo "OUT_PATH is unset or set to the empty string, exit script"; exit; fi


# python file to be called
FILE=MyFirstEvolveableWaterSnake.py

# file name of IN_FILE path
fullfilename=$(basename "$IN_FILE")
# file extension of IN_FILE
extension="${fullfilename##*.}"
# name of IN_FILE
filename="${fullfilename%.*}"

echo "fullfilename: " $fullfilename " extension: " $extension " filename: " $filename

cp $IN_FILE ./$fullfilename
# generates a 'vis_$fullfilename' and a 'vis_$filename.info' file
python $FILE --createvis --input_path=./ > vis_$filename.info

# move the generated files where they belong
mv "vis_$fullfilename" $OUT_PATH
mv "vis_$filename.info" $OUT_PATH

# clean up
rm -f $fullfilename