#!/bin/bash
REMOTE="sunrise@192.168.10.42"
_ENV="env -i HOME=/home/lsy PATH=/usr/bin:/usr/sbin:/bin:/sbin LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu"
RSYNC="$_ENV rsync -avz --progress --delete"
SSH="$_ENV ssh"

# ros_ws src + 脚本
# 本机 src/sllidar_ros2 是软链接，排除掉，由下面单独同步实体目录；
# 不排除的话远端会变成悬空软链接，第二步 mkdir 失败
$RSYNC --exclude /src/sllidar_ros2 \
    /home/lsy/cs_project/robot/ros_ws/src \
    /home/lsy/cs_project/robot/ros_ws/make.sh \
    /home/lsy/cs_project/robot/ros_ws/make_remote.sh \
    $REMOTE:~/max_robot/ros_ws/

# sllidar 实体目录（--delete 自动清理远端多余文件，无需先 ssh rm）
$RSYNC \
    /home/lsy/cs_project/open_source/ros2_2d_slam/src/sllidar_ros2-main/ \
    $REMOTE:~/max_robot/ros_ws/src/sllidar_ros2/
