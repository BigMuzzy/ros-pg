FROM althack/ros2:kilted-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
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
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-tf2-tools \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-rqt-graph ros-${ROS_DISTRO}-rqt-common-plugins \
#    #
#    # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

ARG USERNAME
RUN usermod -aG dialout ${USERNAME}

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
