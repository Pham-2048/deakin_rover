FROM ros:jazzy-ros-base

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

# ============================================================
# ROS 2 packages for rover infrastructure
# ============================================================
RUN sudo apt-get update && sudo apt-get install -y \
    ros-jazzy-rosbridge-server \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-web-video-server \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-xacro \
    ros-jazzy-diagnostic-updater \
    && sudo rm -rf /var/lib/apt/lists/*

# ============================================================
# System tools for monitoring and development
# ============================================================
RUN sudo apt-get update && sudo apt-get install -y \
    # python3-opencv \
    python3-pip \
    python3-numpy \
    python3-yaml \
    wireless-tools \
    net-tools \
    iputils-ping \
    psmisc \
    v4l-utils \
    && sudo rm -rf /var/lib/apt/lists/*

# ============================================================
# Python packages for rover nodes
# ============================================================
RUN pip3 install --no-cache-dir --break-system-packages \
    psutil

# ============================================================
# HARDWARE UPGRADE: Uncomment these when deploying to real rover
# with CAN bus and RS485 hardware attached.
# ============================================================
# RUN sudo apt-get update && sudo apt-get install -y \
#     can-utils \
#     libmodbus-dev \
#     i2c-tools \
#     python3-smbus \
#     && sudo rm -rf /var/lib/apt/lists/*
#
# RUN pip3 install --no-cache-dir --break-system-packages \
#     pymodbus>=3.0 \
#     python-can>=4.0

# Auto-source workspace overlay if it exists
RUN echo '[ -f ~/rover_ws/install/setup.bash ] && source ~/rover_ws/install/setup.bash' >> ~/.bashrc
