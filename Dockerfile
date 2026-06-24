FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

SHELL ["/bin/bash", "-c"]

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-setuptools \
    python3-colcon-common-extensions \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs-py \
    ros-humble-vision-msgs \
    ros-humble-tf-transformations \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    "numpy<2" \
    scipy \
    pillow

# Install graspnetAPI
# RUN pip3 install --no-cache-dir graspnetAPI

RUN mkdir -p /ros2_ws/src /tmp/roslog && \
    echo '. /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then . /ros2_ws/install/setup.bash; fi' >> /root/.bashrc && \
    echo "alias sh='source /opt/ros/humble/setup.bash'" >> /root/.bashrc && \
    echo "alias sw='source /ros2_ws/install/setup.bash'" >> /root/.bashrc && \
    echo 'function cw() { cd /ros2_ws; }' >> /root/.bashrc && \
    echo 'function cs() { cd /ros2_ws/src; }' >> /root/.bashrc && \
    echo 'function cb() { cd /ros2_ws; colcon build --symlink-install; . install/setup.bash; }' >> /root/.bashrc

WORKDIR /ros2_ws

CMD ["/bin/bash"]