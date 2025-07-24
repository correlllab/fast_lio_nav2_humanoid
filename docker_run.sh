#!/bin/bash

docker run --rm -it \
  --shm-size=1g \
  --network host \
  --privileged \
  --name nav2_humble \
  -e QT_X11_NO_MITSHM=1 \
  --env="DISPLAY=$DISPLAY" \
  --env="LD_LIBRARY_PATH=/usr/local/lib" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  nav2:humble \
  /bin/bash
