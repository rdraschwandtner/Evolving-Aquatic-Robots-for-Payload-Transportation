#!/bin/bash

#RUN=ann_mus_worm.py
#RUN=ind_ann_mus_worm.py
RUN=ann_only_worm.py

NUM_JOINTS=$3

for i in $(eval echo {$1..$2}) 
do
    ./run_launcher.sh $i 1000 50 $RUN $NUM_JOINTS 
done
