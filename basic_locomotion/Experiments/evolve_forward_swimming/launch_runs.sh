#!/bin/bash
# call syntay: ./launch_runs.sh [startreplicatenumber] [endreplicatenumber] [svnversion]


# python file to be called
RUN=main.py

REV="$3" #take param 3

SCRATCH_PATH="/scratch/rened/experiments/evolve_forward_swimming"
NOW=$(date +"%m-%d-%Y-%H-%M-%S")

mkdir -p $SCRATCH_PATH/$REV/$NOW #create directory if it does not exist
echo "create out dir " $SCRATCH_PATH/$REV/$NOW


for i in $(eval echo {$1..$2}) 
do
    ./run_launcher.sh $i 1000 50 $RUN $SCRATCH_PATH/$REV/$NOW
done