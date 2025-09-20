FROM node:20

# Install git and chromium
RUN apt-get update && apt-get install -y git chromium

# Clone and build webviz
RUN git clone https://github.com/cruise-automation/webviz.git /webviz
WORKDIR /webviz
RUN npm config set legacy-peer-deps true
ENV PUPPETEER_EXECUTABLE_PATH=/usr/bin/chromium
RUN npm run bootstrap
ENV NODE_OPTIONS=--openssl-legacy-provider
RUN npm run build-static-webviz
RUN ls 
# Set the working directory for the dashboard app
WORKDIR /
