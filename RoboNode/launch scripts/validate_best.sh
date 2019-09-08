#!/bin/bash
RUN_NUM=( 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 )
GENS=(678 519 912 567 168 942 884 990 943 675 857 691 963 996 994 673 986 937 406 634 )
EVAL_TIME=10
NUM_JOINTS=10
FILE=ann_mus_worm.py

for i in {0..19}
do
	python $FILE --log_frames --no_periodic --run_num="${RUN_NUM[$i]}" --num_joints="$NUM_JOINTS" --output_path=/scratch/moore112/experiments/ANN_Mus_Mod_Worm/ --validator --eval_time=$EVAL_TIME --gens=${GENS[$i]}
	mv "${RUN_NUM[$i]}"_logged_output.dat ~/robo_exp/ANN_Mus_Mod_Worm/validation_logging/body_positions/"${RUN_NUM[$i]}"_logged_output_gen_${GENS[$i]}.dat
	rm "${RUN_NUM[$i]}"_joint_angles.dat
done
