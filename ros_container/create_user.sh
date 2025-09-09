#!/bin/bash

# Set timezone
export TZ=Europe/Berlin
ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Set up locale (if not already done in base image)
if ! locale -a | grep -q en_US.utf8; then
    locale-gen en_US en_US.UTF-8
fi
export LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8

# Create group and DOCKER_USER if not exist
if ! getent group $GROUP_ID >/dev/null; then
    groupadd -g $GROUP_ID $DOCKER_USER || true
fi
if ! id -u $DOCKER_USER >/dev/null 2>&1; then
     useradd -ms /bin/bash -u $USER_ID -g $GROUP_ID $DOCKER_USER 

    # useradd -ms /bin/bash -u $USER_ID $USER -g $GROUP_ID || true
fi
usermod -a -G video $DOCKER_USER || true
usermod -a -G sudo $DOCKER_USER || true
echo "$DOCKER_USER:$DOCKER_USER" | chpasswd || true

# Create working directories if needed
for d in src launch config scripts data; do
    mkdir -p /home/$DOCKER_USER/$d
done
chown -R $DOCKER_USER:$GROUP_ID /home/$DOCKER_USER
