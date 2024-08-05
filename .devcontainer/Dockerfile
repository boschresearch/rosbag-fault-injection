FROM osrf/ros:galactic-desktop-focal

SHELL ["/bin/bash", "-c"]

# create a non-root user
# https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

# update all packages
RUN sudo apt-get update && sudo apt-get upgrade -y

# install packages
RUN sudo apt-get install -y python3-pip git

# install fast rtps
RUN sudo apt install -y ros-galactic-rmw-fastrtps-cpp

# update ros dependencies
RUN rosdep update --rosdistro galactic

# ros configuration
RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc