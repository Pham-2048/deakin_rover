FROM osrf/ros:jazzy-desktop-full

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

# Installing Nodejs
RUN sudo curl -fsSL https://deb.nodesource.com/setup_24.x | sudo bash - && \
    sudo apt-get install -y nodejs

EXPOSE 3000

# Block A — ROS 2 arm/MoveIt/ros2_control packages
RUN sudo apt-get update && sudo apt-get install -y \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    && sudo rm -rf /var/lib/apt/lists/*

# Block B — colcon build tools
RUN sudo apt-get update && sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    && sudo rm -rf /var/lib/apt/lists/*

# Block C — X11/OpenGL for RViz2 / MoveIt GUI on Windows
RUN sudo apt-get update && sudo apt-get install -y \
    libglx-mesa0 \
    libgl1-mesa-dri \
    x11-apps \
    mesa-utils \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    && sudo rm -rf /var/lib/apt/lists/*

# Block D — CAN bus utilities (candump/cansend for protocol debugging, ip link for bus setup)
RUN sudo apt-get update && sudo apt-get install -y \
    can-utils \
    iproute2 \
    && sudo rm -rf /var/lib/apt/lists/*
