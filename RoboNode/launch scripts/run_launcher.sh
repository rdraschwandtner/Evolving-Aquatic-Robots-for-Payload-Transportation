EVAL_TIME=10

RUN_NUM="$1"
GENS="$2"
VAL_INTERVAL="$3"
FILE="$4"
NUM_JOINT="$5"

python $FILE --no_periodic --run_num=$RUN_NUM --gens=$GENS --pop_size=120 --eval_time="$EVAL_TIME" --num_joints="$NUM_JOINT" --fit_func="Distance"  --window_size=0.0  --output_path=/scratch/moore112/experiments/ANN_Mus_Mod_Worm_Efficiency/ > /scratch/moore112/experiments/ANN_Mus_Mod_Worm_Efficiency/"$RUN_NUM".out &
wait
./evolutionary_progression_logging.sh $RUN_NUM 50 $GENS $VAL_INTERVAL $FILE $EVAL_TIME $NUM_JOINT > /scratch/moore112/experiments/ANN_Mus_Mod_Worm_Efficiency/"$1"_validator.out
cd ~/robo_exp/ANN_Mus_Mod_Worm_Efficiency/
mkdir "$1"
mkdir "$1"/validator_logging
mv mus_node* "$1"
mv NEAT*"$1"* "$1"
mv "$1"* "$1"
mv validator* "$1"/validator_logging/
mv validation_logging/"$1"/* "$1"/validator_logging/
mv best_individuals* "$1"
mv run* "$RUN_NUM"
rm -rf validation_logging
exit
