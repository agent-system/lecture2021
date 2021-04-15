#!/bin/bash

## install docker compose
COMP_VER=$(docker-compose --version | sed -e 's@.*version \(.*\),.*@\1@')
VER_MID=$(echo "${COMP_VER}" | sed -e 's@.*\.\([0-9]*\)\..*@\1@')

if [ ${VER_MID} -le 17 ]; then
   sudo apt remove docker-compose
   LATEST_COMP_VER=$(wget https://api.github.com/repos/docker/compose/releases/latest -O - | grep 'tag_name' | cut -d\" -f4)
   sudo wget https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-`uname -s`-`uname -m` -O /usr/local/bin/docker-compose
fi
