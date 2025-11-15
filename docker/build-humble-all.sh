#!/bin/bash

sed 's/^export ROS_DISTRO=.*/export ROS_DISTRO=humble/' noros.bashrc.in > noros.bashrc
sed 's/^export ROS_DISTRO=.*/export ROS_DISTRO=humble/' ros.bashrc.in > ros.bashrc

docker build -t ros-humble-all-desktop -f Dockerfile.ros.humble.all .

