#!/bin/bash

#
#
mv docker-compose-robot-assembler.yaml ../robot_assembler
chmod a+x run_roboasm.sh
chmod a+x exec_roboasm.sh
mv run_roboasm.sh  ../robot_assembler
mv exec_roboasm.sh ../robot_assembler

#
#
mv docker-compose-webots.yaml ../webots
chmod a+x run_webots.sh
chmod a+x exec_webots.sh
mv run_webots.sh  ../webots
mv exec_webots.sh ../webots
