# Minimal Sourcing Kit

This project provides a robust and modular framework for data sourcing, focusing on a clean API architecture, containerization, and easy extensibility. It aims to streamline the process of integrating various sensors (e.g., cameras, Ouster lidar) and managing data, including ROS bag analysis.

## Project Goals

*   **Modular API:** Develop a flexible and robust API for managing sourcing processes, including sensor control and data analysis.
*   **Clean Architecture:** The FastAPI application is now decoupled from `rclpy` and communicates with ROS2 via `roslibpy` and `rosbridge_server`.
*   **Extensibility:** Design the system to be easily extensible, allowing for the integration of new sensors, data processing modules, and features.


## Architecture Overview: Process Model
**How does communication work?**

Communication between the isolated FastAPI application and the driver subprocesses is handled by the **ROS 2 middleware**, specifically via `rosbridge_server`:

-   The main FastAPI application does not interact with the hardware directly. Instead, classes like `GigECameraNode` act as **ROS 2 clients**.
-   These clients use the `roslibpy` library to communicate over WebSockets with a `rosbridge_server`.
-   The `rosbridge_server` acts as a bridge, translating these WebSocket messages into standard ROS 2 topics, services, and actions.
-   When an API endpoint needs to set a camera feature, it calls a method on the client class (`GigECameraNode`). This method, in turn, makes a **ROS 2 service call** to the actual driver node running in the separate process.


| Why Subprocesses over Threads? | Bypassing the Global Interpreter Lock (GIL) | Independent Environments |
| --- | --- | --- |
|  Each driver runs in its own isolated memory space. If a driver script encounters a critical error or crashes (which can happen with hardware-specific libraries), it will not affect the main FastAPI server. The API remains stable and can even be designed to manage and restart the failed driver process. In a threaded model, a similar crash would likely terminate the entire application due to shared memory. |  Python's GIL restricts a single process from executing Python bytecode on multiple cores simultaneously. While many driver tasks are I/O-bound, they can involve CPU-intensive operations (like image processing). By using separate processes, each driver can run on a different CPU core, allowing for true parallelism and making full use of the system's resources without being bottlenecked by the GIL | Each subprocess can be configured with its own environment. This is particularly crucial for integrating with ROS (Robot Operating System). The driver scripts often require a specific ROS environment, which is typically set up by sourcing a setup script (e.g., `source /opt/ros/humble/setup.bash`). This is cleanly and reliably managed on a per-process basis using `subprocess`, a task that is complex and error-prone to handle within a single multi-threaded process. |


## Project Structure

### Main
The main project structure includes the following top-level directories and files:

```
minimal_sourcing_kit/
│
├── .gitignore                  # Specifies intentionally untracked files to ignore by Git.
├── docker-compose.yaml         # Defines and runs multi-container Docker applications.
├── Makefile                    # Automates build, test, and deployment tasks.
├── README.md                   # This file, providing an overview of the project.
├── config/                     # Centralized configuration files for various components.
├── data/                       # Directory for storing ROS bag files and other sensor data.
```

### Backend (ros_container)
The backend components, including the API and ROS2 integration, are located within the `ros_container` directory:

```
ros_container/
│
├── build.bash              # Script for building the ROS application container.
├── create_user.sh          # Script for user creation within this container.
├── Dockerfile              # Dockerfile for building the ROS application container.
├── start_services.sh       # Script to start ROS services within the container.
└── api/                    # FastAPI application for API endpoints.
    └── src/                # Source code for the FastAPI application.
        ├── main.py         # Main Python script for the FastAPI server.
        ├── devices/        # Device drivers (e.g., `gig_e_driver.py` using `roslibpy`).
        ├── routers/        # FastAPI routers for different API modules.
        │   └── api.py      # Main router that includes all other API routers.
        ├── server/         # Server-side logic for handling API requests.
        └── utils/          # Utility scripts.
```

### Frontend (dashboard_container)
The frontend application, built with Next.js, is located in the `dashboard_container` directory:

```
dashboard_container/
│
├── Dockerfile              # Dockerfile for building the dashboard container.
├── package.json            # Node.js package definitions and scripts.
├── src/                    # Source code for the dashboard application.
│   ├── app/                # Next.js app directory for routing and components.
│   │   ├── components/     # Reusable UI components.
│   │   ├── live/           # Live data visualization page.
│   │   ├── ouster/         # Ouster lidar data visualization page.
│   │   ├── record/         # Data recording control page.
│   │   ├── replay/         # Data replay control page.
│   │   └── webviz/         # Webviz integration page.
│   └── ...
└── ...

# The frontend application has been refactored. Please refer to the `dashboard_container/src` directory for details.
```

### ROS Base Container (ros_base_container)
The base Docker container with ROS2 and core system dependencies:

```
ros_base_container/
│
├── Dockerfile              # Dockerfile for building the ROS base image.
└── requirements/           # Dependency files for the ROS base container.
    ├── apt_requirements.txt # APT packages required for ROS2.
    ├── pip_requirements.txt # Python packages required for ROS2.
    └── repos.yml           # ROS2 repositories configuration.
```


## API Documentation

The project provides a clear and well-documented API for interacting with its components. The API logic resides within the `ros_container/api/src` directory and is built with FastAPI, featuring a modular design with multiple routers for different functionalities. Communication with ROS2 is handled via `roslibpy` and `rosbridge_server`.

### API Modules

The API is organized into the following modules, each handling a specific set of functionalities:

*   **Camera Management (`camera_management`):** Manages camera-related operations, such as creating, configuring, and deleting camera nodes.
*   **Configuration Management (`configuration_management`):** Handles the loading and saving of system and device configurations.
*   **Driver Management (`driver_management`):** Manages device drivers and their lifecycle.
*   **Feature Control (`feature_control`):** Provides endpoints for controlling device features in real-time.
*   **Logs (`logs`):** Manages system and device logs.
*   **Preset Management (`preset_management`):** Handles the creation, loading, and saving of device presets.
*   **Recording Management (`recording_management`):** Manages ROS bag recording and playback.
*   **Terminal (`terminal`):** Provides a web-based terminal for interacting with the system.

Detailed API documentation, including request/response schemas and examples, can be accessed through the auto-generated OpenAPI/Swagger UI when the application is running.

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
  Builds the main application container, layering your code and scripts on top of the a base image. You can run this after `make build_ros_base` if you don't want to use Docker Compose.

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
This will build and start all containers, set up the network, and mount the required volumes as defined in `docker-compose.yaml`. The `ros_container` will now automatically install and launch `rosbridge_server` on startup.

#### Mounting Scripts and Volumes
- The `ros_container/api` folder is mounted into the `ros_container` at `/home/sourcingapi/ros2_ws/src/api`.
- The `start_services.sh` script is executed as the entrypoint for `ros_container` and now handles `rosbridge_server` setup and FastAPI launch.
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
