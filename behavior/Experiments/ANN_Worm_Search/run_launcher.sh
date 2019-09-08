# call syntax: ./run_launcher.sh [runnumbers] [numberofgenerations] [validationintervall] [filename]
# validationintervall .. eg. 50 look at every 50th individual

EVAL_TIME=30

RUN_NUM="$1"
GENS="$2"
VAL_INTERVAL="$3"
FILE="$4"

OUT_PATH="$5"

python $FILE --run_num=$RUN_NUM --gens=$GENS --pop_size=120 --aquatic --eval_time="$EVAL_TIME" --output_path=$OUT_PATH/ > $OUT_PATH/"$RUN_NUM".out
exit
