#!/bin/bash
REMOTE="sunrise@192.168.10.42"
RSYNC="env -i HOME=/home/lsy PATH=/usr/bin:/usr/sbin:/bin:/sbin LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu rsync -avz --progress --no-times"
SSH="env -i HOME=/home/lsy PATH=/usr/bin:/usr/sbin:/bin:/sbin LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu ssh"

# ros_ws src + 脚本
$RSYNC \
    /home/lsy/cs_project/robot/ros_ws/src \
    /home/lsy/cs_project/robot/ros_ws/make.sh \
    /home/lsy/cs_project/robot/ros_ws/make_remote.sh \
    $REMOTE:~/max_robot/ros_ws/

# sllidar 实体目录（远程可能是符号链接，先删再传）
$SSH $REMOTE "rm -rf ~/max_robot/ros_ws/src/sllidar_ros2"
$RSYNC \
    /home/lsy/cs_project/open_source/ros2_2d_slam/src/sllidar_ros2-main/ \
    $REMOTE:~/max_robot/ros_ws/src/sllidar_ros2/
