#!/bin/bash
set -e

export BASE_IMAGE=jskrobotics/agent_system:ros_melodic_2021
TEMP_IMAGE=temp_robot_assembler:latest
## output: jskrobotics/agent_system:robot_assembler_2021

### main build
if [ ! -e build ]; then
    mkdir build
fi
cd build

if [ ! -e Dockerfile.assembler ]; then
    wget https://raw.githubusercontent.com/agent-system/robot_assembler/master/docker/Dockerfile.assembler -O Dockerfile.assembler
fi

docker build . --no-cache -f Dockerfile.assembler --tag ${TEMP_IMAGE} --build-arg BASE_IMAGE=${BASE_IMAGE}

### wrap for xserver
if [ -e robot_assembler_wrapper ]; then
    rm -rf robot_assembler_wrapper
fi
git clone --depth=1 -b xserver_nvidia https://github.com/YoheiKakiuchi/misc_docker.git robot_assembler_wrapper

cd robot_assembler_wrapper

./wrap_for_docker_xserver.sh ${TEMP_IMAGE} --ros --user-directory '/jskrobotics'

# fix docker-compose
sed -i -e "s@#command: // should be set at supervisord.conf@command: [bash, -c, 'until xset q; do sleep 3; done \&\& roslaunch robot_assembler kxr_assembler.launch']@" docker-compose.yaml
cp docker-compose.yaml ../../docker-compose-robot-assembler.yaml
# fix environment?

# to build (dir)
cd ..

### tag for output image
docker tag ${TEMP_IMAGE}_xserver jskrobotics/agent_system:robot_assembler_2021

### for run/exec
if [ ! -e ../run_roboasm.sh ]; then
    wget https://github.com/YoheiKakiuchi/misc_docker/raw/master/run.sh  -O ../run_roboasm.sh
fi
if [ ! -e ../exec_roboasm.sh ]; then
    wget https://github.com/YoheiKakiuchi/misc_docker/raw/master/exec.sh -O ../exec_roboasm.sh
fi

# remove temp images
docker image rm ${TEMP_IMAGE}
docker image rm ${TEMP_IMAGE}_xserver
