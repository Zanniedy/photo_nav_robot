#!/bin/bash
env -i \
    HOME=/home/lsy \
    PATH=/usr/bin:/usr/sbin:/bin:/sbin \
    LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu \
    rsync -avz --progress \
    /home/lsy/cs_project/robot/ros_ws/src \
    /home/lsy/cs_project/robot/ros_ws/make.sh \
    sunrise@192.168.10.42:~/max_robot/ros_ws/
