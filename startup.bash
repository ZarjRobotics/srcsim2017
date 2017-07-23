#!/bin/bash

source /opt/ros/indigo/setup.sh
source /home/docker/ws/install/setup.bash

while [ 1 ] ; do
    /usr/bin/python ./fc/fc.py >> /tmp/fc.log 2>&1
    sleep 60
done
