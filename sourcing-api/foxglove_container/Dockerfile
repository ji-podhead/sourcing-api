# Use a Debian-based image as recommended by Foxglove
FROM ubuntu:22.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages for downloading and installing .deb files
RUN apt-get update && apt-get install -y \
    wget \
    gnupg \
    lsb-release \
    libgtk-3-0 \
    libnotify4 \
    libnss3 \
    libxtst6 \
    xdg-utils \
    libatspi2.0-0 \
    libdrm2 \
    libgbm1 \
    libxcb-dri3-0 \
    libappindicator3-1 \
    libasound2 \
    libasound \
    && rm -rf /var/lib/apt/lists/*
# Add Foxglove GPG key
RUN wget https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-amd64.deb
RUN apt install ./foxglove-studio-*.deb
# Install Foxglove Studio
RUN  apt update 
RUN apt install foxglove-studio



# Set up a non-root user
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID foxglove_user && useradd -m -s /bin/bash -u $UID -g $GID foxglove_user
USER foxglove_user
WORKDIR /home/foxglove_user

# Expose the default Foxglove Studio port (if it has one for external connections, though typically it's a desktop app)
# Foxglove Studio is primarily a desktop application, but if it were to expose a port for e.g. WebSocket connections,
# you would expose it here. For now, we'll assume it's for running the GUI.
# If you intend to run it headless and connect to it, you might need to expose a specific port.
# For now, we'll just run the application.

# Command to run Foxglove Studio
CMD ["foxglove-studio"]
