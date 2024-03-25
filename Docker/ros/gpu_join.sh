#!/usr/bin/env bash
#
# Typical usage: ./join.bash subt
#

IMG=argnctu/robotx2022:ubuntu-20.04

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}")&& echo $containerid
docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES="$(tput lines)" -it ${containerid} bash
xhost -