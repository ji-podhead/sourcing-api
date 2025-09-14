# Load environment variables from .env file if it exists
ifneq (,$(wildcard ./.env))
    include .env
    export
endif

# Default target
all: up

# Build targets
build: build_ros_base build_ros_container
	@echo "All images built successfully."

build_ros_base:
	@echo "Building ROS base container..."
	docker compose build ros_base_container
	@echo "ROS base container built."

build_ros_container:
	@echo "Building ROS container..."
	docker compose build ros_container
	@echo "ROS container built."

# Bring up all services defined in docker-compose.yaml
up: build
	@echo "Starting services with docker compose..."
	docker compose up -d
	@echo "Services started."

# Stop all services defined in docker-compose.yaml
down:
	@echo "Stopping services..."
	docker compose down
	@echo "Services stopped."

# Clean up Docker images and volumes
clean:
	@echo "Stopping and cleaning up services, images, and volumes..."
	docker compose down --rmi all -v --remove-orphans
	@echo "Cleanup complete."

.PHONY: all build build_ros_base build_ros_container up down clean
