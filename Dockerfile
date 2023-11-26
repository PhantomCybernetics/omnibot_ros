ARG ROS_DISTRO=iron

FROM ros:$ROS_DISTRO

RUN apt-get update -y --fix-missing
RUN apt-get install -y ssh \
                       vim mc \
                       iputils-ping net-tools iproute2 curl \
                       pip

RUN pip install --upgrade pip
RUN pip install setuptools==58.2.0 \
                termcolor \
                PyEventEmitter \
                pyserial

RUN apt-get install -y ros-$ROS_DISTRO-joint-state-publisher \
		               ros-$ROS_DISTRO-xacro \
                       ros-$ROS_DISTRO-ros2-control \
                       ros-$ROS_DISTRO-ros2-controllers \
                       ros-$ROS_DISTRO-realtime-tools
#RUN apt-get install -y ros-$ROS_DISTRO-navigation2 \
#		       ros-$ROS_DISTRO-nav2-bringup

# init workspace
ENV ROS_WS /ros2_ws
RUN mkdir -p $ROS_WS/src

# generate entrypoint script
RUN echo '#!/bin/bash \n \
set -e \n \
\n \
# setup ros environment \n \
source "/opt/ros/'$ROS_DISTRO'/setup.bash" \n \
test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash" \n \
\n \
exec "$@"' > /ros_entrypoint.sh

RUN chmod a+x /ros_entrypoint.sh

# source underlay on every login
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /root/.bashrc
RUN echo 'test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash"' >> /root/.bashrc

WORKDIR $ROS_WS

# TODO install package
# pull omnibot repo into $ROS_WS/src/omnibot
# RUN rosdep install -i --from-path src --rosdistro iron -y
# RUN colcon build --packages-select omnibot

# pimp up prompt with hostame and color
RUN echo "PS1='\${debian_chroot:+(\$debian_chroot)}\\[\\033[01;35m\\]\\u@\\h\\[\\033[00m\\] \\[\\033[01;34m\\]\\w\\[\\033[00m\\] 🤖 '"  >> /root/.bashrc


ENTRYPOINT ["/ros_entrypoint.sh"]
CMD [ "bash" ]
