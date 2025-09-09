# Load environment variables from .env file
export $(shell sed 's/=.*//' .env)

# Project root directory
PROJECT := $(realpath .)

# User and group IDs for container creation
# Use the environment variables defined in the .env file

# Dockerfile paths
ROS_BASE_CONTAINER_DOCKERFILE := $(PROJECT)/ros_base_container/Dockerfile
ROS_CONTAINER_DOCKERFILE := $(PROJECT)/ros_container/Dockerfile
DASHBOARD_CONTAINER_DOCKERFILE := $(PROJECT)/dashboard_container/Dockerfile

# Docker image names
ROS_BASE_CONTAINER_NAME := ros_base_container:latest
ROS_CONTAINER_NAME := minimal_sourcing_kit-ros_container
DASHBOARD_CONTAINER_NAME := minimal_sourcing_kit-dashboard_container

# Project directories
CONTAINER_FILES := $(PROJECT)/project/container
LAUNCH_FILES := $(PROJECT)/launch
CONFIG := $(PROJECT)/config
SCRIPTS := $(PROJECT)/scripts
SCRIPTS_CONTAINER := $(SCRIPTS)/container
RECORDINGS := $(PROJECT)/recordings

# Build the ROS base container image
# This image contains ROS Humble, Python dependencies, and common tools.
build_ros_base:
	@echo "Building ROS base container image..."
	docker build \
		--build-arg USER=$$(DOCKER_USER) \
		--build-arg USER_ID=$(USER_ID) \
		--build-arg GROUP_ID=$(GROUP_ID) \
		-f $(ROS_BASE_CONTAINER_DOCKERFILE) \
		-t $(ROS_BASE_CONTAINER_NAME) \
		.
	@echo "ROS base container image built successfully."

# Build the ROS container image using the base image
# This image adds application-specific code and scripts.
build_ros_container: build_ros_base
	@echo "Building ROS container image..."
	docker build \
		--build-arg USER=$(USER) \
		--build-arg USER_ID=$(USER_ID) \
		--build-arg GROUP_ID=$(GROUP_ID) \
		-f $(ROS_CONTAINER_DOCKERFILE) \
		-t $(ROS_CONTAINER_NAME) \
		--build-arg BASE_IMAGE=$(ROS_BASE_CONTAINER_NAME) \
		.
	@echo "ROS container image built successfully."

# Build the dashboard container image
build_dashboard_container:
	@echo "Building dashboard container image..."
	docker build \
		--build-arg USER=$(USER) \
		--build-arg USER_ID=$(USER_ID) \
		--build-arg GROUP_ID=$(GROUP_ID) \
		-f $(DASHBOARD_CONTAINER_DOCKERFILE) \
		-t $(DASHBOARD_CONTAINER_NAME) \
		$(PROJECT)/dashboard_container
	@echo "Dashboard container image built successfully."

# User creation target
create_user:
	@echo "Creating user in ROS container..."
	./create_user.sh
build_images: build_ros_base build_ros_container build_dashboard_container

# Bring up all services defined in docker-compose.yaml
up: build_images
	@echo "Starting services with docker compose..."
	docker compose up -d
	@echo "Services started. Access the dashboard at http://localhost:3000"

# Stop all services defined in docker-compose.yaml
down:
	@echo "Stopping services..."
	docker compose down
	@echo "Services stopped."

# Clean up Docker images
clean_images:
	@echo "Cleaning up Docker images..."
	docker rmi $(ROS_BASE_CONTAINER_NAME) || true
	docker rmi $(ROS_CONTAINER_NAME) || true
	docker rmi $(DASHBOARD_CONTAINER_NAME) || true
	@echo "Docker images cleaned up."

# Clean up Docker volumes (use with caution)
# clean_volumes:
# 	@echo "Cleaning up Docker volumes..."
# 	docker volume rm $(docker volume ls -qf) || true
# 	@echo "Docker volumes cleaned up."

# Default target
all: up
