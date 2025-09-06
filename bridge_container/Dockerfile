# Schritt 1: Beginnen Sie mit einem offiziellen ROS 1 Noetic Image (basiert auf Ubuntu 20.04)
FROM ros:noetic-ros-core

# Schritt 2: Fügen Sie die ROS 2 Foxy Repositories hinzu (Foxy ist die korrekte ROS-2-Version für Ubuntu 20.04)
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Schritt 3: Installieren Sie ROS 2 Foxy und die ros1_bridge
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base \
    ros-foxy-ros1-bridge \
    && rm -rf /var/lib/apt/lists/*

# Schritt 4: Kopieren Sie das Entrypoint-Skript, das beide Umgebungen sourct
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
