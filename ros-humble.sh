#!/bin/bash
# Usage: $SCRIPT [restart] [--ros-dist <jazzy|humble>] [--ssh-port <PORT>] [--grpc-port <PORT>]

cd $(dirname $(readlink -f $0))

#HOSTNAME=$(hostname)
#DISPLAY_ID=${DISPLAY#:}
#FULL_DISPLAY=$(xauth list | grep "unix" | awk '{print $1}')
#COOKIE=$(xauth list | grep "unix" | awk '{print $3}')

# ROS_DISTRO=jazzy
ROS_DISTRO=humble
# SSH_PORT=22
SSH_PORT=10022
# GRPC_PORT=50051
GRPC_PORT=50052

SPEC_IMAGE=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ros-dist)
            ROS_DISTRO="$2"
            shift 2
            ;;
        --ssh-port)
            SSH_PORT="$2"
            shift 2
            ;;
        --grpc-port)
            GRPC_PORT="$2"
            shift 2
            ;;
        restart)
            COMMAND="restart"
            shift
            ;;
        *)
            if [ -z "$SPEC_IMAGE" ]; then
                SPEC_IMAGE="$1"
            else
                echo "Error: unknown argument '$1'"
                exit 1
            fi
            shift
            ;;
    esac
done

XAUTH=/tmp/.docker.ros.${ROS_DISTRO}.desktop.xauth
#IMAGE_NAME="${SPEC_IMAGE:="ros-${ROS_DISTRO}-gzb-desktop:latest"}"
IMAGE_NAME="${SPEC_IMAGE:="ros-${ROS_DISTRO}-all-desktop:latest"}"
CONT_NAME=ros-${ROS_DISTRO}-desktop-container-edu

echo -e "\n------ Configurations ------"
echo -e "Command: ${COMMAND:-Default Start}"
echo -e "ROS Distro: $ROS_DISTRO"
echo -e "SSH Port: $SSH_PORT"
echo -e "GRPC Port: $GRPC_PORT"
echo -e "Image Name: $IMAGE_NAME"
echo -e "Container Name: $CONT_NAME"
echo -e "----------------------------\n"

touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

if [ "$COMMAND" = "restart" ]; then
    echo "Restarting container $CONT_NAME..."
    docker restart $CONT_NAME
    docker exec -it $CONT_NAME bash
else
    docker run -it \
        -v $PWD:/root/workspace \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e XAUTHORITY=$XAUTH \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $XAUTH:$XAUTH:ro \
        -p $SSH_PORT:22 \
        -p $GRPC_PORT:50051 \
        -p 8000:8000 \
        -p 8001:8001 \
        -p 8002:8002 \
        --device /dev/dri \
        --name $CONT_NAME \
        $IMAGE_NAME bash -c "echo -e \"export DISPLAY=${DISPLAY};\nexport QT_X11_NO_MITSHM=1;\nexport XAUTHORITY=${XAUTH};\nexport MUJOCO_VER=3.3.4;\nexport MUJOCO_DIR=/root/workspace/mujoco/\\\${MUJOCO_VER};\nexport PATH=\\\${PATH}:/root/workspace/blender-app:\\\${MUJOCO_DIR}/bin;\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;\n\" >> /root/.bashrc && /usr/sbin/sshd && /usr/bin/bash --rcfile /root/.bashrc"

fi

rm -f $XAUTH

