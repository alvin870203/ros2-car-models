# FROM ros:humble
FROM ros:iron
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID alvin \
    && useradd --uid $USER_UID --gid $USER_GID -m alvin \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo alvin ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/alvin \
    && chmod 0440 /etc/sudoers.d/alvin
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip

# Install gazebo classic
RUN apt-get install -y curl
RUN curl -sSL http://get.gazebosim.org | sh
# Install common ros2 packages
# RUN apt-get install -y ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-rviz2 \
#                        ros-humble-gazebo-ros-pkgs ros-humble-ros2-control ros-humble-ros2-controllers \
#                        ros-humble-gazebo-ros2-control ros-humble-effort-controllers ros-humble-teleop-twist-keyboard \
#                        ros-humble-topic-tools ros-humble-rqt*
RUN apt-get install -y ros-iron-xacro ros-iron-joint-state-publisher-gui ros-iron-rviz2 \
                       ros-iron-gazebo-ros-pkgs ros-iron-ros2-control ros-iron-ros2-controllers \
                       ros-iron-gazebo-ros2-control ros-iron-effort-controllers ros-iron-teleop-twist-keyboard \
                       ros-iron-topic-tools ros-iron-rqt*

ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

################
# Expose the nvidia driver to allow opengl
# Dependencies for glvnd and X11.
################
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER alvin
CMD ["/bin/bash"]
