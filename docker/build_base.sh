#!/bin/bash
set -e

## base_image: nvidia/cudagl:10.0-devel-ubuntu18.04
export BASE_IMAGE=nvidia/opengl:1.2-glvnd-runtime-ubuntu18.04

##
cd build

docker pull ${BASE_IMAGE}

## base_ubuntu
## output: jskrobotics/agent_system:ubuntu18.04_2021

docker image tag ${BASE_IMAGE} jskrobotics/agent_system:ubuntu18.04_2021


## output: jskrobotics/agent_system:ros_melodic_2021
docker build . --no-cache -f ../dockerfiles/Dockerfile.ros_from_base --build-arg BASE_IMAGE=${BASE_IMAGE} -t jskrobotics/agent_system:ros_melodic_2021

##
cd ..
