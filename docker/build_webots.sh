#!/bin/bash
set -e

export BASE_IMAGE=jskrobotics/agent_system_ros_melodic:2021
TEMP_IMAGE=temp_webots:latest
## output: jskrobotics/agent_system_webots:2021

### main build
cd build

wget https://github.com/cyberbotics/webots-docker/raw/master/Dockerfile -O Dockerfile.webots-docker

docker build . --file Dockerfile.webots-docker --tag ${TEMP_IMAGE} --build-arg BASE_IMAGE=${BASE_IMAGE} --build-arg WEBOTS_PACKAGE_PREFIX=_ubuntu-18.04

### wrap for xserver
git clone --depth=1 -b xserver_nvidia https://github.com/YoheiKakiuchi/misc_docker.git webots_wrapper

cd webots_wrapper

./wrap_for_docker_xserver.sh ${TEMP_IMAGE} --ros --user-directory '/jskrobotics'

# fix docker-compose
sed -i -e "s@#command: // should be set at supervisord.conf@command: [bash, -c, 'until xset q; do sleep 3; done \&\& webots']@" docker-compose.yaml
cp docker-compose.yaml ../../docker-compose-webots.yaml

# to build (dir)
cd ..

### buid for urdf2webots
docker build . --no-cache -f ../dockerfiles/Dockerfile.urdf2webots --build-arg BASE_IMAGE=${TEMP_IMAGE}_xserver --tag jskrobotics/agent_system_webots:2021

### for run/exec
wget https://github.com/YoheiKakiuchi/misc_docker/raw/master/run.sh  -O ../run_webots.sh
wget https://github.com/YoheiKakiuchi/misc_docker/raw/master/exec.sh -O ../exec_webots.sh
## image: ubuntu:18.04 ->
## container: misc_docker ->
## /userdir -> /jskrobotics

# remove temp images
docker image rm ${TEMP_IMAGE}
docker image rm ${TEMP_IMAGE}_xserver
cd ..
