#!/bin/bash

RUN_NUM="$1"
SGEN_INTERVAL="$2"
GENS="$3"
INTERVAL="$4"
FILE="$5"
EVAL_TIME="$6"
NUM_JOINTS="$7"

LAST_GEN=$(($GENS-1))

mkdir ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/validation_logging
mkdir ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/validation_logging/"$RUN_NUM"
mkdir ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/validation_logging/"$RUN_NUM"/body_positions
mkdir ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/validation_logging/"$RUN_NUM"/joint_angles

# Get the top 10 individuals from all runs.
#top_10=`python node_position_novelty_sorter.py --run_num="$RUN_NUM"`

python $FILE --log_frames --no_periodic --run_num="$RUN_NUM" --num_joints="$NUM_JOINTS" --fit_func="Distance" --window_size=0.0 --output_path=/scratch/moore112/experiments/ANN_Mus_Mod_Worm_Efficiency/ --validator  --eval_time=$EVAL_TIME --gens=0
mv "$RUN_NUM"_logged_output.dat ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/validation_logging/"$RUN_NUM"/body_positions/"$RUN_NUM"_logged_output_gen_0.dat
mv "$RUN_NUM"_joint_angles.dat ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/validation_logging/"$RUN_NUM"/joint_angles/"$RUN_NUM"_joint_angles_gen_0.dat
for j in $(eval echo {$SGEN_INTERVAL..$(($GENS-$INTERVAL))..$INTERVAL})
do
    echo $j
    python $FILE --log_frames --no_periodic --run_num="$RUN_NUM" --num_joints="$NUM_JOINTS" --fit_func="Distance" --window_size=0.0 --output_path=/scratch/moore112/experiments/ANN_Mus_Mod_Worm_Efficiency/ --validator  --eval_time=$EVAL_TIME --gens=$j 
    mv "$RUN_NUM"_logged_output.dat ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/validation_logging/"$RUN_NUM"/body_positions/"$RUN_NUM"_logged_output_gen_"$j".dat
    mv "$RUN_NUM"_joint_angles.dat ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/validation_logging/"$RUN_NUM"/joint_angles/"$RUN_NUM"_joint_angles_gen_"$j".dat
done
python $FILE --log_frames --no_periodic --run_num="$RUN_NUM" --num_joints="$NUM_JOINTS" --fit_func="Distance" --window_size=0.0 --output_path=/scratch/moore112/experiments/ANN_Mus_Mod_Worm_Efficiency/ --validator  --eval_time=$EVAL_TIME --gens="$LAST_GEN" 
mv "$RUN_NUM"_logged_output.dat ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/validation_logging/"$RUN_NUM"/body_positions/"$RUN_NUM"_logged_output_gen_"$LAST_GEN".dat
mv "$RUN_NUM"_joint_angles.dat ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/validation_logging/"$RUN_NUM"/joint_angles/"$RUN_NUM"_joint_angles_gen_"$LAST_GEN".dat
