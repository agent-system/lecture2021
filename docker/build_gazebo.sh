#!/bin/bash

export BASE_IMAGE=jskrobotics/agent_system:ros_melodic_2021
TEMP_IMAGE=temp_gazebo:latest
## output: jskrobotics/agent_system:gazebo11_dart_2021

### main build
if [ ! -e build ]; then
    mkdir build
fi
cd build

if [ ! -e Dockerfile.gazebo_dart ]; then
    wget https://raw.githubusercontent.com/YoheiKakiuchi/misc_docker/build_gazebo/Dockerfile -O Dockerfile.gazebo_dart
fi

docker build . --no-cache -f Dockerfile.gazebo_dart --tag ${TEMP_IMAGE} --build-arg BASE_IMAGE=${BASE_IMAGE}


### wrap for xserver
if [ -e gazebo_wrapper ]; then
    rm -rf gazebo_wrapper
fi
git clone --depth=1 -b xserver_nvidia https://github.com/YoheiKakiuchi/misc_docker.git gazebo_wrapper

cd gazebo_wrapper

./wrap_for_docker_xserver.sh ${TEMP_IMAGE} --ros

# fix docker-compose
#sed -i -e "s@#command: // should be set at supervisord.conf@command: [bash, -c, 'until xset q; do sleep 3; done \&\& rosrun gazebo_ros gazebo ']@" docker-compose.yaml
#cp docker-compose.yaml ../../docker-compose-gazebo.yaml
## devert/xserver -> jskrobotics/agent_system:xserver_2021
## ${TEMP_IMAGE}_xserver -> output
## /userdir -> /jskrobotics

# to build (dir)
cd ..

### tag for output image
docker tag ${TEMP_IMAGE}_xserver jskrobotics/agent_system:gazebo11_dart_2021

### for run/exec
if [ ! -e ../run_gazebo.sh ]; then
    wget https://github.com/YoheiKakiuchi/misc_docker/raw/master/run.sh  -O ../run_gazebo.sh
fi
if [ ! -e ../exec_gazebo.sh ]; then
    wget https://github.com/YoheiKakiuchi/misc_docker/raw/master/exec.sh -O ../exec_gazebo.sh
fi

#docker image rm ${TEMP_IMAGE}
#docker image rm ${TEMP_IMAGE}_xserver
