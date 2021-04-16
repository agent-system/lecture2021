#!/bin/bash
set -e

## base_image: nvidia/cudagl:10.0-devel-ubuntu18.04
export BASE_IMAGE=nvidia/opengl:1.2-glvnd-runtime-ubuntu18.04
TEMP_IMAGE=temp_ros:latest

##
if [ ! -e build ]; then
    mkdir build
fi
cd build

docker pull ${BASE_IMAGE}

## base_ubuntu
## output: jskrobotics/agent_system:ubuntu18.04_2021

docker image tag ${BASE_IMAGE} jskrobotics/agent_system:ubuntu18.04_2021


## output: jskrobotics/agent_system:ros_melodic_2021

docker build . --no-cache -f ../dockerfiles/Dockerfile.ros_from_base --build-arg BASE_IMAGE=${BASE_IMAGE} -t ${TEMP_IMAGE}

if [ ! -e Dockerfile.virtualgl ]; then
    wget https://github.com/YoheiKakiuchi/misc_docker/raw/xserver_nvidia/Dockerfile.virtualgl -O Dockerfile.virtualgl
fi

docker build . --no-cache -f Dockerfile.virtualgl --build-arg BASE_IMAGE=${TEMP_IMAGE} -t jskrobotics/agent_system:ros_melodic_2021

##
docker image rm ${TEMP_IMAGE}
