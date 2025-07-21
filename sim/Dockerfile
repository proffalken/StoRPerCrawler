FROM ros:kilted

# Install dependencies and add Ignition apt source
RUN apt-get update && apt-get install -y \
    wget gnupg lsb-release curl \
 && curl -sSL https://packages.osrfoundation.org/gazebo.key \
    | gpg --dearmor > /usr/share/keyrings/gazebo-archive-keyring.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Install Ignition (Gazebo) Ionic and ROS 2 simulation tools
RUN apt-get update && apt-get install -y \
    gz-ionic \
    ros-kilted-robot-state-publisher \
    ros-kilted-joint-state-publisher-gui \
    ros-kilted-teleop-twist-keyboard \
    ros-kilted-ros-gz \
    ros-kilted-gz-ros2-control \
    ros-kilted-gz-tools-vendor \
    ros-kilted-teleop-twist-keyboard && \
    #ros-kilted-gz-sim \
    #ros-kilted-gz-msgs && \
    rm -rf /var/lib/apt/lists/*

# Setup ROS 2 workspace
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Auto-source ROS 2 setup in shell
SHELL ["/bin/bash", "-lc"]
RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]

