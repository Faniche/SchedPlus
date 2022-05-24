#!/bin/bash

solution_dir="/home/faniche/Projects/TSN/SchedPlus/cmake-build-debug/xml/small"

if [ $1 == 0 ]; then
  upload_xmls=$(ls $solution_dir/0*.xml)
  for file in $upload_xmls
    do
      docker cp $file nesting:/root/models/nesting/simulations/schedplus/xml/
    done
  ini_file=$(ls $solution_dir/0*.ini)
  docker cp $ini_file nesting:/root/models/nesting/simulations/schedplus/
fi
