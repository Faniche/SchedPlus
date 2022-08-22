#!/bin/bash

solution_dir="/home/faniche/Projects/TSN/SchedPlus/cmake-build-debug/xml/small/wait"
result_bak_dir="/home/faniche/Projects/TSN/SchedPlus/cmake-build-debug/result_bak"

#if [ $# == 1 ]; then
#  upload_xmls=$(ls $solution_dir/$1*.xml)
#  for file in $upload_xmls
#    do
#      docker cp $file nesting:/root/models/nesting/simulations/schedplus/xml/
#    done
#  ini_file=$(ls $solution_dir/$1*.ini)
#  docker cp $ini_file nesting:/root/models/nesting/simulations/schedplus/
#fi

# $1: flow_num
# $2: wait, nowait
# $3: topology
# $4: test_idx
# $5: solution_idx
upload_xmls=$(ls $result_bak_dir/$1/$2/$3/$4/$5_*.xml)
for file in $upload_xmls
  do
    docker cp $file nesting:/root/models/nesting/simulations/schedplus/xml/
  done
ini_file=$(ls $result_bak_dir/$1/$2/$3/$4/$5_*.ini)
docker cp $ini_file nesting:/root/models/nesting/simulations/schedplus/