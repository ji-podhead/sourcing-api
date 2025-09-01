# Build ROS 2 Humble base image manually (no official image for ARM64v8)
FROM ubuntu:22.04

ARG USER
ARG USER_ID
ARG GROUP_ID
ARG DEBIAN_FRONTEND=noninteractive
ARG SOURCE=/home/$USER/src

SHELL ["/bin/bash", "-c"]

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
RUN apt-get update && apt-get upgrade -y

# Install all required libraries (from apt_requirements.txt, updated for humble)
COPY ros_base_container/requirements/apt_requirements.txt /tmp/apt_requirements.txt
RUN apt-get update && \
    xargs -a /tmp/apt_requirements.txt apt-get install -y --no-install-recommends && \
    apt-get remove -y meson && \
    rm /tmp/apt_requirements.txt && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install a newer version of meson via pip
RUN pip3 install --no-cache-dir meson

# Upgrade pip, setuptools, and wheel before installing Python requirements
RUN pip3 install --upgrade pip setuptools wheel

# Install Python requirements from pip_requirements.txt
COPY ros_base_container/requirements/pip_requirements.txt /tmp/pip_requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/pip_requirements.txt && rm /tmp/pip_requirements.txt


# Set up user (robust, avoid duplicate UID/GID)
RUN if ! getent group $GROUP_ID >/dev/null; then groupadd -g $GROUP_ID iamsourcing; fi \
    && if ! id -u $USER >/dev/null 2>&1; then useradd -ms /bin/bash -u $USER_ID $USER -g $GROUP_ID; fi \
    && usermod -a -G video $USER \
    && usermod -a -G sudo $USER \
    && echo "$USER:$USER" | chpasswd || true


# Imaging Source utilities and tiscamera (installed via requirements file)

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone


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

# install tiscamera
# WORKDIR /repos/tiscamera
# RUN ./scripts/dependency-manager install
# RUN mkdir build
# WORKDIR /repos/tiscamera/build
# RUN cmake ..
# RUN make -j
# RUN mkdir /opt/tiscamera
# RUN mv /repos/tiscamera/build/bin/* /usr/bin
# RUN mv /repos/tiscamera/build/lib/* /usr/lib
# RUN rm -r /repos/tiscamera
