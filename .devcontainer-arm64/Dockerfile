FROM arm64v8/ros:kilted-perception

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
# Install additional packages not included in perception image
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-xacro \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-joint-state-publisher-gui \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-ros-gz \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-ros2-control \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-ros2-controllers \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-moveit \
   && apt-get -y install --no-install-recommends libserial-dev \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-navigation2 \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-nav2-bringup \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-turtlebot3* \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-nav2-simple-commander \
   && apt-get -y install --no-install-recommends x11-xserver-utils \
   && apt-get -y install --no-install-recommends x11-apps \
   && apt-get -y install --no-install-recommends python3-pip \
   && apt-get -y install --no-install-recommends python3-dev \
   && apt-get -y install --no-install-recommends build-essential \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-twist-mux \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-joy \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-teleop-twist-joy \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-teleop-twist-keyboard \
#    #
#    # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Check if "ubuntu" user exists, delete it if it does, then create the desired user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"

# Add sudo support for the non-root user
RUN apt-get update && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Set up autocompletion for user
RUN apt-get update && apt-get install -y --no-install-recommends git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/*

ARG USERNAME
RUN usermod -aG dialout ${USERNAME}

# Install sudo for devcontainer user management
RUN apt-get update && apt-get install -y sudo && apt-get clean


# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc