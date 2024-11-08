#!/bin/bash

OUT=${1:-out}

mkdir -p $OUT

parallel --bar --eta -j1 --shuf --header : \
   "roslaunch moveit_task_constructor_demo pickplace.launch execute:=false keep_running:=false connect_parallel_attempts:={workers} workers:={workers} > $OUT/pick_and_place_{workers}workers_rep{repetition}.log 2>&1" \
   ::: workers $(seq -1 7) \
   ::: repetition $(seq 10)
