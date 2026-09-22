FROM osrf/ros:humble-desktop-full@sha256:1db1e4e941d4f77fab55bcd479158a75273329335e86ec468bff280859e8178c

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    WORKSPACE=/workspace/ros2_ws \
    UV_LINK_MODE=copy

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libglew-dev \
    libglfw3-dev \
    libxi-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxrandr-dev \
    libboost-all-dev \
    libfmt-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    mesa-utils \
    nano \
    python3-colcon-common-extensions \
    python3-jsonschema \
    python3-pip \
    python3-rosdep \
    python3-yaml \
    python3-venv \
    python3-vcstool \
    ros-humble-ament-cmake-gmock \
    ros-humble-ament-lint-common \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-diagnostic-updater \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-vision-msgs \
    ros-humble-joint-state-publisher \
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-xacro \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/*

COPY docker/requirements.txt /tmp/geoff-requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/geoff-requirements.txt

RUN if ! getent group ${USER_GID} >/dev/null; then groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && usermod -aG sudo,video,dialout ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN rosdep init || true

COPY docker/scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
WORKDIR /workspace

RUN rosdep update || true \
    && mkdir -p /workspace/ros2_ws/src /workspace/vendor

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
