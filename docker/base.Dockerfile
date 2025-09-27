FROM --platform=$BUILDPLATFORM ubuntu:22.04

ARG ROS_DISTRO=humble
ARG USERNAME=dee
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN sed -i 's/# \(.*universe\)/\1/' /etc/apt/sources.list

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    build-essential \
    cmake \
    curl \
    gcc \
    git \
    python3-dev \
    python3-pip \
    software-properties-common \
    sudo \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN sudo apt-get update && sudo apt-get upgrade -y \
    && sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    ros-${ROS_DISTRO}-ros-base \
    ros-dev-tools \
    && sudo rm -rf /var/lib/apt/lists/*

RUN echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\"" >> "/home/${USERNAME}/.bashrc" 

CMD ["/bin/bash"]
