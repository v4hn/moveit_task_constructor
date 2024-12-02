#!/bin/bash

OUT=${1:-out}

mkdir -p $OUT

parallel --bar --eta -j3 --shuf --header : \
   "roslaunch moveit_task_constructor_demo pickplace.launch execute:=false keep_running:=false name:=mtc_tutorial{#} max_solutions:={solutions} connect_compute_attempts:={workers} workers:={workers} > $OUT/pick_and_place_{workers}workers_{solutions}solutions_rep{repetition}.log 2>&1" \
   ::: workers -1 $(seq 1 15) \
   ::: repetition $(seq 10) \
   ::: solutions 1 10 100 1000
