#!/bin/bash
set -e

export BASE_IMAGE=jskrobotics/agent_system_ros_melodic:2021
TEMP_IMAGE=temp_robot_assembler:latest
## output: jskrobotics/agent_system_robot_assembler:2021

### main build
cd build

wget https://raw.githubusercontent.com/agent-system/robot_assembler/master/docker/Dockerfile.assembler -O Dockerfile.assembler

docker build . --no-cache -f Dockerfile.assembler --tag ${TEMP_IMAGE} --build-arg BASE_IMAGE=${BASE_IMAGE}

### wrap for xserver
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
docker tag ${TEMP_IMAGE}_xserver jskrobotics/agent_system_robot_assembler:2021

### for run/exec
wget https://github.com/YoheiKakiuchi/misc_docker/raw/master/run.sh  -O ../run_roboasm.sh
wget https://github.com/YoheiKakiuchi/misc_docker/raw/master/exec.sh -O ../exec_roboasm.sh

# remove temp images
docker image rm ${TEMP_IMAGE}
docker image rm ${TEMP_IMAGE}_xserver
cd ..
