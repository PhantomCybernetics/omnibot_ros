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
    image: omnibot:iron
    container_name: omnibot
    hostname: omnibot.local
    restart: unless-stopped
    privileged: true
    volumes:
      # - ~/omnibot:/ros2_ws/src/omnibot
      # - ~/omnibot.yaml:/ros2_ws/omnibot.yaml
      - ~/omnibot_ws:/ros2_ws
    devices:
      - /dev/cu.usbserial-0001:/dev/serial0 # serial port to esp32
    command:
      /bin/sh -c "while sleep 1000; do :; done"
      #/bin/sh /phntm_bridge_ui/run.web-ui.sh
```

