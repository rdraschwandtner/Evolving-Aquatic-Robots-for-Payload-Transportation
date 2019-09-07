# call syntax: ./run_launcher.sh [runnumbers] [numberofgenerations] [validationintervall] [filename]
# validationintervall .. eg. 50 look at every 50th individual

EVAL_TIME=10

RUN_NUM="$1"
GENS="$2"
VAL_INTERVAL="$3"
FILE="$4"

OUT_PATH="$5"

python $FILE --run_num=$RUN_NUM --gens=$GENS --pop_size=120 --num_joints=16 --eval_time="$EVAL_TIME" --output_path=$OUT_PATH/ > $OUT_PATH/"$RUN_NUM".out &
wait
./evolutionary_progression_logging.sh $RUN_NUM 50 $GENS $VAL_INTERVAL $FILE $EVAL_TIME $OUT_PATH/ > $OUT_PATH/"$1"_validator.out
cd $OUT_PATH
mkdir "$1"
mkdir "$1"/validator_logging
mv "$1"* "$1"
mv validation_logging/"$1"/* "$1"/validator_logging/
mv best_individuals* "$1"
mv run* "$RUN_NUM"
rm -rf validation_logging
rm -rf validator_logging
exit
