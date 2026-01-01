FROM --platform=$BUILDPLATFORM ghcr.io/droneepicengineering/base:latest

ARG USERNAME=dee
ARG ROS_DISTRO=humble

USER ${USERNAME}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV ROS_WS=/home/${USERNAME}/ws
ENV REPO_DIR_NAME=mission-control

WORKDIR ${ROS_WS}/src
RUN git clone "https://github.com/PX4/px4_msgs" --branch "release/1.15"
RUN git clone "https://github.com/eProsima/Micro-XRCE-DDS-Agent" --branch v2.4.1

RUN sudo apt-get update && sudo apt-get -y --quiet --no-install-recommends install \
    ros-${ROS_DISTRO}-rviz2 \
    && sudo rm -rf /var/lib/apt/lists/*

RUN sudo pip3 install -U "numpy==2.0.2" "matplotlib==3.10.0" "pandas==2.2.2" "typing-extensions==4.12.2"

RUN sudo apt-get update && sudo apt-get -y --quiet --no-install-recommends install \
    ros-${ROS_DISTRO}-py-trees \
    ros-${ROS_DISTRO}-py-trees-ros-interfaces \
    ros-${ROS_DISTRO}-py-trees-ros \
    ros-${ROS_DISTRO}-py-trees-ros-viewer \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_WS}
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build

WORKDIR ${ROS_WS}/src
COPY --chown=${USERNAME} . ${REPO_DIR_NAME}

WORKDIR ${ROS_WS}/src/${REPO_DIR_NAME}
RUN sudo rm -rf build install log
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    "${ROS_WS}/src/${REPO_DIR_NAME}/scripts/build.bash"

RUN echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "source \"${ROS_WS}/install/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "source \"${ROS_WS}/src/${REPO_DIR_NAME}/install/setup.bash\"" >> "/home/${USERNAME}/.bashrc"

CMD ["/bin/bash"]
