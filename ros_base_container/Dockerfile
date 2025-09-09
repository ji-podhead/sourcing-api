FROM ubuntu:22.04

ARG DOCKER_USER=sourcingapi
ARG USER_ID=1010
ARG GROUP_ID=1000
ARG DEBIAN_FRONTEND=noninteractive
ARG SOURCE=/home/$DOCKER_USER/src
SHELL ["/bin/bash", "-c"]
# ----------------------------- > Base Setup < ----------------------------- #
# Set up locales
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8
# Add universe repo and curl
RUN apt-get install -y software-properties-common \
    && add-apt-repository universe
RUN apt-get update && apt-get install -y curl
# Add ROS 2 Humble repo and key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
# Copy apt_requirements.txt
COPY ros_base_container/requirements/apt_requirements.txt /tmp/apt_requirements.txt
# Update apt and install packages
RUN apt-get update && xargs -a /tmp/apt_requirements.txt apt-get install -y --no-install-recommends && apt-get clean && rm -rf /var/lib/apt/lists/*
# ----------------------------- > Python Setup < ----------------------------- #
# Install Poetry
RUN pip3 install --no-cache-dir poetry

# ----------------------------- > User Setup < ----------------------------- #
ARG USER=sourcingapi
ARG USER_ID=1010
ARG GROUP_ID=1000
# Set up user (robust, avoid duplicate UID/GID)
RUN groupadd -g $GROUP_ID $USER || true \
    # && useradd -ms /bin/bash -u $USER -g $GROUP_ID $USER \
    && usermod -a -G video $USER \
    && usermod -a -G audio $USER \
    && usermod -a -G sudo $USER \
    && echo "$USER:$USER" | chpasswd || true
ENV HOME=/home/$USER
RUN echo user: $USER id: $USER_ID group: $GROUP_ID
RUN mkdir -p /home/$USER && chown -R $USER:$GROUP_ID /home/$USER
RUN mkdir -p /home/.ros && chown -R $USER:$GROUP_ID /home/.ros
USER $USER
#----------------------------- > Installing Depencies < ----------------------- #
# Copy pyproject.toml
COPY ros_base_container/pyproject.toml /home/$DOCKER_USER/ros2_ws/api/pyproject.toml
WORKDIR /home/$DOCKER_USER/ros2_ws/api
# USER $DOCKER_USER
# Install dependencies
RUN poetry install --no-root --no-interaction --no-ansi
# Set WORKDIR and USER after dependencies are installed
# ----------------------------- > External Repos < --------------------------- #
WORKDIR /repos
# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
# Set locales
RUN locale-gen en_US en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
# Clone and build external repositories
RUN mkdir -p /repos
WORKDIR /repos
COPY ros_base_container/requirements/repos.yml /repos/repos.yml
RUN while read repo; do \
        git clone "$repo"; \
    done < /repos/repos.yml && \
    rm /repos/repos.yml
#----------------------------------------------------------------------------- #
