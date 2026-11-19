# Multi-arch base (linux/amd64 + linux/arm64), so this also builds natively
# on Apple Silicon instead of relying on amd64 emulation.
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Ubuntu and packages.ros.org can be very slow from some networks (e.g.
# China).  These defaults use the Tsinghua mirrors; override them to use the
# upstream servers instead, e.g.
#   docker compose build \
#     --build-arg ROS_APT_MIRROR=http://packages.ros.org/ros2/ubuntu \
#     --build-arg UBUNTU_MIRROR=http://archive.ubuntu.com/ubuntu \
#     --build-arg UBUNTU_PORTS_MIRROR=http://ports.ubuntu.com/ubuntu-ports
ARG ROS_APT_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
ARG UBUNTU_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu
ARG UBUNTU_PORTS_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
RUN sed -i \
        -e "s|http://packages.ros.org/ros2/ubuntu|${ROS_APT_MIRROR}|g" \
        -e "s|^Types: deb deb-src|Types: deb|" \
        /usr/share/ros-apt-source/ros2.sources \
    && sed -i \
        -e "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" \
        -e "s|http://security.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g" \
        -e "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_PORTS_MIRROR}|g" \
        /etc/apt/sources.list

# ros-humble-ros-gz pulls in Gazebo Fortress on Humble.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-default-runtime \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-rviz2 \
        ros-humble-teleop-twist-keyboard \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        python3-colcon-common-extensions \
        mesa-utils \
        libgl1 \
        libgl1-mesa-dri \
        libglx-mesa0 \
        libx11-xcb1 \
        x11-utils \
        xvfb \
        x11vnc \
        xauth \
        fluxbox \
        xterm \
        dbus-x11 \
        novnc \
        python3-websockify \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src /workspace/src

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /usr/local/bin/ros_entrypoint.sh
COPY docker/start_gui.sh /usr/local/bin/start_gui.sh
RUN chmod +x /usr/local/bin/ros_entrypoint.sh /usr/local/bin/start_gui.sh

ENTRYPOINT ["/usr/local/bin/ros_entrypoint.sh"]
CMD ["bash"]
