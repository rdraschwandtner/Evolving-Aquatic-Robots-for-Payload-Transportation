#!/bin/bash

RUN_NUM="$1"
SGEN_INTERVAL="$2"
GENS="$3"
INTERVAL="$4"
FILE="$5"
EVAL_TIME="$6"

OUT_PATH="$7"

LAST_GEN=$(($GENS-1))

mkdir $OUT_PATH/validation_logging
mkdir $OUT_PATH/validation_logging/"$RUN_NUM"
mkdir $OUT_PATH/validation_logging/"$RUN_NUM"/joint_angles

# Get the top 10 individuals from all runs.
#top_10=`python node_position_novelty_sorter.py --run_num="$RUN_NUM"`

python $FILE --log_frames --run_num="$RUN_NUM" --output_path=$OUT_PATH --validator --eval_time=$EVAL_TIME --gens=0
mv "$RUN_NUM"_logged_output.dat $OUT_PATH/validation_logging/"$RUN_NUM"/"$RUN_NUM"_logged_output_gen_0.dat
mv $OUT_PATH/validator_logging/joint_angles/rep"$RUN_NUM"_gen0_joint_angles.dat $OUT_PATH/validation_logging/"$RUN_NUM"/joint_angles/
for j in $(eval echo {$SGEN_INTERVAL..$(($GENS-$INTERVAL))..$INTERVAL})
do
    echo $j
    python $FILE --log_frames --run_num="$RUN_NUM" --output_path=$OUT_PATH --validator --eval_time=$EVAL_TIME --gens=$j
    mv "$RUN_NUM"_logged_output.dat $OUT_PATH/validation_logging/"$RUN_NUM"/"$RUN_NUM"_logged_output_gen_"$j".dat
	mv $OUT_PATH/validator_logging/joint_angles/rep"$RUN_NUM"_gen"$j"_joint_angles.dat $OUT_PATH/validation_logging/"$RUN_NUM"/joint_angles/
done
python $FILE --log_frames --run_num="$RUN_NUM" --output_path=$OUT_PATH --validator --eval_time=$EVAL_TIME --gens="$LAST_GEN"
mv "$RUN_NUM"_logged_output.dat $OUT_PATH/validation_logging/"$RUN_NUM"/"$RUN_NUM"_logged_output_gen_"$LAST_GEN".dat
mv $OUT_PATH/validator_logging/joint_angles/rep"$RUN_NUM"_gen"$LAST_GEN"_joint_angles.dat $OUT_PATH/validation_logging/"$RUN_NUM"/joint_angles/