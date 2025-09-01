# Minimal Sourcing Kit

This project provides a robust and modular framework for ROS2-based data sourcing, focusing on a clean API architecture, containerization, and easy extensibility. It aims to streamline the process of integrating various sensors (e.g., cameras, Ouster lidar) and managing data, including ROS bag analysis.

## Project Goals

*   **Modular API:** Develop a flexible and robust API for managing sourcing processes, including sensor control and data analysis.
*   **Clean Architecture:** Ensure a clear separation of concerns between infrastructure, container orchestration, and application logic for improved maintainability.
*   **Extensibility:** Design the system to be easily extensible, allowing for the integration of new sensors, data processing modules, and features.
*   **Containerization:** Utilize Docker for consistent and isolated development, testing, and deployment environments.

## Project Structure

The `minimal_sourcing_kit` repository is organized as follows:

```
minimal_sourcing_kit/
│
├── .gitignore                  # Specifies intentionally untracked files to ignore by Git.
├── build.bash                  # Script for building the entire project.
├── docker-compose.yaml         # Defines and runs multi-container Docker applications.
├── Makefile                    # Automates build, test, and deployment tasks.
├── README.md                   # This file, providing an overview of the project.
│
├── config/                     # Centralized configuration files for various components.
│   ├── bag_checker_config.yaml # Configuration for ROS bag data integrity checks.
│   ├── imaging_source_camera_info.yaml # Camera calibration and info for Imaging Source cameras.
│   ├── imaging_source_dynamic_parameters.yaml # Dynamic parameters for Imaging Source cameras.
│   ├── imaging_source.yaml     # General configuration for Imaging Source cameras.
│   └── ouster.yaml             # Configuration for Ouster lidar sensors.
│
├── create_user.sh/             # Script for creating a user within the Docker containers.
│
├── dashboard_container/        # Frontend application for monitoring and control (e.g., Next.js).
│   ├── Dockerfile              # Dockerfile for building the dashboard container.
│   ├── package.json            # Node.js package definitions and scripts.
│   ├── src/                    # Source code for the dashboard application.
│   │   ├── app/                # Next.js app directory for routing and components.
│   │   │   ├── components/     # Reusable UI components.
│   │   │   ├── live/           # Live data visualization page.
│   │   │   ├── ouster/         # Ouster lidar data visualization page.
│   │   │   ├── record/         # Data recording control page.
│   │   │   ├── replay/         # Data replay control page.
│   │   │   ├── rgb/            # RGB camera data visualization page.
│   │   │   └── webviz/         # Webviz integration page.
│   │   └── ...
│   └── ...
│
├── data/                       # Directory for storing ROS bag files and other sensor data.
│   ├── metadata.yaml           # Metadata for recorded ROS bags.
│   ├── r2b_galileo_0.mcap      # Example MCAP ROS bag file.
│   └── r2b_hope.db3            # Example SQLite3 ROS bag file.
│
├── ros_base_container/         # Base Docker container with ROS2 and core system dependencies.
│   ├── Dockerfile              # Dockerfile for building the ROS base image.
│   └── requirements/           # Dependency files for the ROS base container.
│       ├── apt_requirements.txt # APT packages required for ROS2.
│       ├── pip_requirements.txt # Python packages required for ROS2.
│       └── repos.yml           # ROS2 repositories configuration.
│
├── ros_container/              # Main application Docker container with ROS2 and custom scripts.
│   ├── build.bash              # Script for building the ROS application container.
│   ├── create_user.sh          # Script for user creation within this container.
│   ├── Dockerfile              # Dockerfile for building the ROS application container.
│   ├── start_services.sh       # Script to start ROS services within the container.
│   ├── scripts/                # Custom scripts for ROS operations and device control.
│   │   ├── devices/            # Scripts for interacting with specific hardware devices.
│   │   │   ├── imaging_source.py # Python script for Imaging Source cameras.
│   │   │   ├── ouster.py       # Python script for Ouster lidar.
│   │   │   └── record.py       # Python script for recording data.
│   │   ├── recording/          # Scripts related to ROS bag recording and playback.
│   │   │   ├── play.bash       # Bash script for playing ROS bags.
│   │   │   └── record.bash     # Bash script for recording ROS bags.
│   │   ├── server/             # Backend server logic (e.g., FastAPI for API endpoints).
│   │   │   └── main.py         # Main Python script for the server.
│   │   └── utils/              # Utility scripts for common tasks.
│   │       ├── create_rosbag_name.py # Utility to generate ROS bag names.
│   │       ├── discover_imaging_source_camera.py # Utility to discover Imaging Source cameras.
│   │       ├── is_online.bash  # Utility to check network connectivity.
│   │       └── ouster_address.py # Utility to get Ouster lidar IP address.
│   └── ...
│
└── scripts/                    # Top-level helper scripts (e.g., for overall project setup).
    └── ...

```

## API Documentation

The project aims to provide a clear and well-documented API for interacting with its components. The primary API logic is intended to reside within the `ros_container/scripts/server/main.py` file, likely utilizing a framework like FastAPI for RESTful endpoints.

### Key API Endpoints (Planned/Example)

*   `/record`: Endpoint to start and stop ROS bag recordings.
*   `/play`: Endpoint to play back ROS bag files.
*   `/status`: Endpoint to retrieve the current status of sensors and recording processes.
*   `/devices`: Endpoint to list and configure connected devices (e.g., cameras, Ouster).
*   `/convert`: Endpoint to convert ROS bag formats (e.g., MCAP to SQLite3).

Detailed API documentation, including request/response schemas and examples, will be generated or maintained alongside the implementation. For Python-based APIs, tools like Sphinx, MkDocs, or OpenAPI/Swagger UI can be integrated for automatic documentation generation.

## Build & Development

## Migration Notes
- The old `project/` folder should be removed and relevant content moved to `src/`.
- API logic should be placed in its own subfolder (e.g. `src/api/`).
- Containerization (Docker) is intended for all components.

## Build & Development

### Build Commands Overview

- **Build only the ROS base container:**
  ```bash
  make build_ros_base
  ```
  Builds the base image with ROS2 and all system dependencies. Use this if you only want to update the base image.

- **Build the application container (after base):**
  ```bash
  make build_ros_container
  ```
  Builds the main application container, layering your code and scripts on top of the base image. You can run this after `make build_ros_base` if you don't want to use Docker Compose.

- **Build the dashboard container:**
  ```bash
  make build_dashboard_container
  ```
  Builds the optional dashboard/frontend container.

- **Build all images at once:**
  ```bash
  make build_images
  ```
  Builds all containers (base, application, dashboard) in the correct order.

### Running with Docker Compose

You can use Docker Compose to start all services at once:
```bash
docker compose up --build
```
This will build and start all containers, set up the network, and mount the required volumes as defined in `docker-compose.yaml`.

#### Mounting Scripts and Volumes
- The `scripts/` folder and other directories are mounted into the containers as defined in `docker-compose.yaml`.
- Example for mounting a script:
  ```yaml
  volumes:
    - ./create_user.sh:/home/create_user.sh
  ```
  This makes `create_user.sh` available at `/home/create_user.sh` inside the container.

#### Working Directory
- The `working_dir` property in the Compose file sets the default working directory for commands (e.g. `/home`).

#### User Creation
- The entrypoint/command in the Compose file runs `create_user.sh` before starting the main service. Make sure the script is executable and correctly sets up the user.

### Alternative: Manual Build & Run
You can also build and run containers individually with the provided Makefile targets, without using Docker Compose. This is useful for development or debugging single services.
- The `scripts/` directory is mounted into the containers via Docker Compose, so you can update scripts without rebuilding the image.
- You can also mount other folders (e.g. configs, data) as needed.

### Manual Container Usage

If you prefer not to use Docker Compose, you can build the containers with the `make` commands above and run them manually with `docker run`, mounting scripts or configs as needed.

## ToDo / Next Steps
- [ ] Remove the `project/` folder and migrate its contents
- [ ] Set up the API base structure in `src/api/`
- [ ] Update the README regularly

---

**Contact:**
For questions or contributions: please open issues or pull requests.
