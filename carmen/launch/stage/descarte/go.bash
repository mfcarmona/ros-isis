#!/bin/bash
rxgraph &
/home/distaur/ros/laser_pipeline/laser_assembler/examples/periodic_snapshotter &
roslaunch stageros_tester.launch


