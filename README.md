# Omnibot V2 ROS Driver

## Omnibot Directory Listing
```
 |- compose.yaml # Docker Compose file, see below
 |
 |- omnibot_ros # Omnibot V2 ROS package (/description , /tf, odometry)
 |   |          # see https://github.com/PhantomCybernetics/omnibot_ros
 |   |- description # (URDFs & meshes live-linked to container for easy edits, just restart)
 |
 |- ld19_lidar # LD19 Lidar fork, see https://github.com/PhantomCybernetics/ld19_lidar
 |
 |- phntm_bridge  # phntm bridge, see https://github.com/PhantomCybernetics/phntm_bridge
 |- phntm_bridge.yaml # phntm bridge config
 |- aioice # temp dev mount (phntm_bridge dep)
 |- aiortc # temp dev mount (phntm_bridge dep)
 |
 |- omnibot_firmware # Omnibot ESP32 micro-ROS firmware, see https://github.com/PhantomCybernetics/omnibot_firmware
```

## Install Docker & Docker Compose

## Build the Image
```bash
docker build -f Dockerfile -t phntm/omnibot-ros:humble .
```

Add the following to you compose.yaml
```yaml
version: "3.9"

services:

  omnibot:
    image: phntm/omnibot-ros:humble
    container_name: omnibot
    hostname: omnibot.local
    restart: unless-stopped
    privileged: false
    environment:
      - TERM=xterm
    volumes:
      - ~/omnibot_ros:/ros2_ws/src/omnibot
    command:
      ros2 launch omnibot omnibot_controllers.launch.py
      # /bin/sh -c "while sleep 1000; do :; done"

  ld19_lidar:
    image: phntm/ld19-lidar:humble
    container_name: ld19-lidar
    hostname: omnibot-lidar.local
    restart: unless-stopped
    privileged: false
    environment:
      - TERM=xterm
    devices:
      - /dev/ttyUSB1:/dev/ld19_lidar
    command: >
      ros2 launch ld19_lidar lidar.launch.py
        topic_name:=/scan frame_id:=base_laser

  omnibot_microros:
    image: microros/micro-ros-agent:humble
    container_name: omnibot-microros
    restart: unless-stopped
    privileged: true
    devices:
      - /dev:/dev
    command: >
      serial --dev /dev/ttyUSB0
  
  phntm_bridge:
    image: phntm/bridge:humble
    container_name: phntm-bridge
    hostname: phntm-bridge.local
    restart: unless-stopped
    privileged: true
    network_mode: host
    cpuset: '0,1,2'
    shm_size: 200m #more room for shared camera frames
    environment:
      - TERM=xterm
    volumes:
      - ~/phntm_bridge.yaml:/ros2_ws/phntm_bridge_params.yaml
      - ~/phntm_bridge:/ros2_ws/src/phntm_bridge
      - ~/aioice:/ros2_ws/aioice
      - ~/aiortc:/ros2_ws/aiortc
      - /var/run:/host_run
      - /tmp:/tmp
    devices:
      - /dev:/dev
    command:
      ros2 launch phntm_bridge bridge_launch.py

