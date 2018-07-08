#!/bin/bash
docker run --rm -i -v `pwd`:`pwd` -e "CI_SOURCE_PATH=`pwd`" -e "ROS_DISTRO=kinetic" -t ubuntu:xenial sh -c "cd `pwd`; ./.travis.sh"
