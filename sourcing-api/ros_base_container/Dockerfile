FROM ubuntu:22.04

ARG DOCKER_USER=sourcingapi
ARG USER_ID=1010
ARG GROUP_ID=1000
ARG DEBIAN_FRONTEND=noninteractive

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
# ----------------------------- > User Setup < ------------------------------- #
# Create a non-root user
RUN groupadd -g $GROUP_ID $DOCKER_USER || true \
    && useradd -ms /bin/bash -u $USER_ID -g $GROUP_ID -m $DOCKER_USER \
    && usermod -aG sudo,video,audio $DOCKER_USER \
    && echo "$DOCKER_USER:$DOCKER_USER" | chpasswd
# Set home environment
ENV HOME=/home/$DOCKER_USER
WORKDIR $HOME
# ----------------------------- > Timezone < ---------------------------------- #
# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
# Set locales
RUN locale-gen en_US en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8
#----------------------------- > Installing Depencies < ----------------------- #
# Switch to the new user
USER $DOCKER_USER
# Copy pyproject.toml and install dependencies
COPY --chown=$DOCKER_USER:$GROUP_ID ros_base_container/pyproject.toml $HOME/ros2_ws/api/pyproject.toml
WORKDIR $HOME/ros2_ws/api
RUN poetry install --no-root --no-interaction --no-ansi
COPY . .
# ----------------------------- > External Repos < --------------------------- #
WORKDIR /repos
# Clone and build external repositories
RUN mkdir -p /repos
WORKDIR /repos
COPY --chown=$DOCKER_USER:$GROUP_ID ros_base_container/requirements/repos.yml /repos/repos.yml
RUN while read repo; do \
        git clone "$repo"; \
    done < /repos/repos.yml && \
    rm /repos/repos.yml
#----------------------------------------------------------------------------- #
WORKDIR $HOME
