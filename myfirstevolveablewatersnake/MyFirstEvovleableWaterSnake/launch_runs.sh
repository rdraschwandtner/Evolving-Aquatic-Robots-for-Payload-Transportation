#!/bin/bash
# call syntay: ./launch_runs.sh [startreplicatenumber] [endreplicatenumber] [source code revision]

# clean:
# 1) find . | grep 'snake_gen' | xargs rm -f
# 2) rm -rf /scratch/rened/experiments/MyFirstEvovleableWaterSnake/*

# mkdir /scratch/rened/ 
# mkdir /scratch/rened/experiments/
# mkdir /scratch/rened/experiments/MyFirstEvovleableWaterSnake/


# python file to be called
FILE=MyFirstEvolveableWaterSnake.py

SCRATCH_PATH="/scratch/rened/experiments/MyFirstEvovleableWaterSnake"

REV="$3" #take param 3
mkdir -p $SCRATCH_PATH/$REV #create directory if it does not exist

NOW=$(date +"%m-%d-%Y-%H-%M-%S")
mkdir $SCRATCH_PATH/$REV/$NOW


for i in $(eval echo {$1..$2}) 
do
	OUT=$SCRATCH_PATH/$REV/$NOW/$i
	mkdir $OUT	
	
	# this call generates snake_gen*_indvidx*.json files in this directory
	# replicate number is used as seed
	python $FILE --pop_size=120 --gens=1000 --seed=$i --output_path=$OUT/ > "$OUT.out"
	# resolve filename with index wildcard in name
	bestindvfilename="$(ls -d $OUT/snake_gen999_bestindvidx*.json)"
	./createvisualization.sh $OUT/$bestindvfilename $OUT/
done
