FROM ros_base_container:latest

ARG USER=sourcingapi
ARG USER_ID=1010
ARG GROUP_ID=1000
ARG SOURCE=/home/$USER/src
# ARG BUILD=/home/$USER/build.bash # This ARG is now redundant as we copy directly to the final path
ARG SCRIPTS=scripts
RUN echo "User: $USER, UID: $USER_ID, GID: $GROUP_ID"
RUN useradd -ms /bin/bash  $USER \
    && usermod -a -G video $USER \
    && usermod -a -G audio $USER \
    && usermod -a -G sudo $USER \
    && echo "$USER:$USER" | chpasswd || true

SHELL ["/bin/bash", "-c"]
