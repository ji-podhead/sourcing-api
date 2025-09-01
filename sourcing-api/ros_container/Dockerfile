FROM ros_base_container:latest

ARG USER
ARG USER_ID
ARG GROUP_ID
ARG SOURCE=/home/$USER/src
# ARG BUILD=/home/$USER/build.bash # This ARG is now redundant as we copy directly to the final path
ARG SCRIPTS=scripts

SHELL ["/bin/bash", "-c"]

#Set up user (already done in base image, but good to ensure consistency if base image changes)
# RUN groupadd -g $GROUP_ID iamsourcing
# RUN useradd -ms /bin/bash -u $USER_ID $USER -g $GROUP_ID
# RUN usermod -a -G video $USER
# RUN usermod -a -G sudo $USER
# RUN echo "$USER:$USER" | chpasswd

RUN apt-get update && apt-get install -y ros-humble-camera-aravis2 netcat-openbsd postgresql-client && rm -rf /var/lib/apt/lists/*

# Installations-Script kopieren
# COPY install_aravis2.sh ./install_aravis2.sh

# Setuptools auf Version 58.2.0 setzen (ROS 2 kompatibel)
RUN pip install --no-cache-dir setuptools==58.2.0

# Installations-Script ausführen
# RUN bash ./install_aravis2.sh && rm ./install_aravis2.sh
RUN pip install uvicorn[standard]
# Copy application-specific code and scripts
# The build context is now the project root ('.') due to docker-compose.yaml change
# COPY --chown=$USER_ID:$GROUP_ID ros_container/main.py /home/$USER/main.py
# COPY --chown=$USER_ID:$GROUP_ID ros_container/launch/ /home/$USER/launch/
# COPY --chown=$USER_ID:$GROUP_ID config/ /home/$USER/config/
# # COPY --chown=$USER_ID:$GROUP_ID scripts/container/ /home/$USER/scripts/container/
# COPY --chown=$USER_ID:$GROUP_ID ros_container/scripts/devices/ /home/$USER/scripts/devices/
# # Explicitly copy build.bash from the project root context
# COPY --chown=$USER_ID:$GROUP_ID ./build.bash /home/$USER/build.bash
#RUN chown $USER_ID:$GROUP_ID /home/$USER/build.bash
 RUN apt update 
    
RUN apt install -y ros-humble-foxglove-bridge
RUN pip install numpy==1.26.4
RUN pip install xmltodict
  
WORKDIR /home/$USER

# Build the sources (source code is mounted via docker-compose volumes)
#RUN /home/$USER/build.bash

# Clean up
# RUN rm -r log
