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
#    #
#    # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Create ros user for ARM64 (similar to althack image)
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Add to dialout group for serial access
RUN usermod -aG dialout ${USERNAME}

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "source /opt/ros/kilted/setup.bash" >> /home/ros/.bashrc
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# Switch to ros user
USER $USERNAME