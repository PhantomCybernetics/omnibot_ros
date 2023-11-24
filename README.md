# Omnibot ROS Driver

## Install Docker & Dockerc Compose

## Buid Container
```bash
docker build -f Dockerfile -t omnibot:iron .
```

Add the following to you compose.yaml
```yaml
services:
  omnibot:
    image: omnibot/firmware:iron
    container_name: omnibot
    hostname: omnibot.local
    restart: unless-stopped
    privileged: false
    environment:
      - TERM=xterm
    volumes:
      # - ~/omnibot_firmware:/ros2_ws/src/omnibot
      # - ~/omnibot_firmware.yaml:/ros2_ws/omnibot.yaml
      - ~/omnibot_ws:/ros2_ws
      # devices:
      # - /dev/ttyUSB0:/dev/ttyUSB0
    command:
      /bin/sh -c "while sleep 1000; do :; done"
      #/bin/sh /phntm_bridge_ui/run.web-ui.sh

  omnibot_microros:
    image: microros/micro-ros-agent:iron
    container_name: omnibot-microros
    restart: unless-stopped
    privileged: true
    devices:
      - /dev:/dev
    command:
      serial --dev /dev/ttyUSB0
  
  omnibot_bridge:
    image: phntm/bridge:iron
    container_name: omnibot-bridge
    hostname: omnibot-bridge.local
    restart: unless-stopped
    privileged: true
    network_mode: host
    cpuset: '0,1,2'
    shm_size: 200m #more room for shared camera frames
    environment:
      - TERM=xterm
    volumes:
      - ~/phntm_bridge:/ros2_ws/src/phntm_bridge
      - ~/omnibot_bridge_params.yaml:/ros2_ws/phntm_bridge_params.yaml
      - /var/run:/host_run
      - /tmp:/tmp
      - ~/aioice:/ros2_ws/aioice
      - ~/aiortc:/ros2_ws/aiortc
    devices:
      - /dev:/dev
    command:
      ros2 launch phntm_bridge bridge_launch.py
```

