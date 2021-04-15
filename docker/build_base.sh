#!/bin/bash
set -e

## base_image: nvidia/cudagl:10.0-devel-ubuntu18.04
export BASE_IMAGE=nvidia/opengl:1.2-glvnd-runtime-ubuntu18.04

##
cd build

docker pull ${BASE_IMAGE}

## base_ubuntu
## output: jsk_robotics/agent_system_ubuntu18.04:2021

docker image tag ${BASE_IMAGE} jsk_robotics/agent_system_ubuntu18.04:2021


## output: jsk_robotics/agent_system_ros_melodic:2021
docker build . --no-cache -f ../dockerfiles/Dockerfile.ros_from_base --build-arg BASE_IMAGE=${BASE_IMAGE} -t jsk_robotics/agent_system_ros_melodic:2021

##
cd ..
