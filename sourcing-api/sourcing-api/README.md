# Sourcing API

This project provides a robust and modular framework for data sourcing, focusing on a clean API architecture, containerization, and easy extensibility. It aims to streamline the process of integrating various sensors (e.g., cameras, Ouster lidar) and managing data, including ROS bag analysis.

## Project Goals

*   **Automatic Camera Discovery & State:** Added automatic camera discovery and state management for seamless integration.
*   **Modular API:** Develop a flexible and robust API for managing sourcing processes, including sensor control and data analysis.
*   **Clean Architecture:** The FastAPI application is now decoupled from `rclpy` and communicates with ROS2 via `roslibpy` and `rosbridge_server`.
*   **Extensibility:** Design the system to be easily extensible, allowing for the integration of new sensors, data processing modules, and features.

---

## Build & Development
### Running with Docker Compose

You can use Docker Compose to buid and start all services at once:
```bash
docker compose up --build
```
This will build and start all containers, set up the network, and mount the required volumes as defined in `docker-compose.yaml`. The `ros_container` will now automatically install and launch `rosbridge_server` on startup.

### Make Targets

The `Makefile` provides several targets to simplify common development tasks:

*   **all:** Builds and starts all services using Docker Compose. This is the default target.
*   **build:** Builds all necessary Docker images.
*   **build_ros_base:** Builds the ROS base container.
*   **build_ros_container:** Builds the ROS container.
*   **build_dashboard_container:** Builds the dashboard container.
*   **up:** Starts all services defined in `docker-compose.yaml` in detached mode.
*   **down:** Stops all running services defined in `docker-compose.yaml`.
*   **clean:** Stops and removes all services, images, and volumes defined in `docker-compose.yaml`.


**Build and start all containers as background process (Production):**
   ```bash
  make up
   ```
---

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
├── Dockerfile              # Dockerfile for building the ROS application container.
├── init.sql                # SQL script for database initialization.
├── start_services.sh       # Script to start ROS services within the container.
├── api/                    # FastAPI application for API endpoints.
│   └── src/                # Source code for the FastAPI application.
│       ├── main.py         # Main Python script for the FastAPI server.
│       ├── debug.py        # Script for debugging purposes.
│       ├── print_device_attributes.py # Script to print device attributes.
│       ├── devices/        # Device drivers (e.g., `gig_e_driver.py` using `roslibpy`).
│       │   ├── __init__.py
│       │   ├── gig_e_driver.py
│       │   └── ouster/     # Ouster lidar device drivers.
│       │       ├── ouster.py
│       │       └── record.py
│       ├── routers/        # FastAPI routers for different API modules.
│       │   └── api.py      # Main router that includes all other API routers.
│       ├── server/         # Server-side logic for handling API requests.
│       │   ├── camera_management.py
│       │   ├── configuration_management.py
│       │   ├── driver_management.py
│       │   ├── feature_control.py
│       │   ├── logs.py
│       │   ├── preset_management.py
│       │   ├── recording_management.py
│       │   ├── state.py
│       │   └── terminal.py
│       └── utils/          # Utility scripts.
│           ├── db/         # Database utilities.
│           │   └── db_utils.py
│           ├── gigE/       # GigE camera utilities.
│           │   ├── camera_features.py
│           │   └── camera_manager.py
│           ├── gmsl2/      # GMSL2 camera utilities.
│           ├── logs/       # Logging utilities.
│           │   └── logs.py
│           └── recording/  # Recording utilities.
│               ├── create_rosbag_name.py
│               ├── play.bash
│               └── record.bash
└── ros/                    # ROS2 package for services.
    ├── package.xml
    ├── README.md
    ├── setup.py
    ├── resource/
    │   └── api
    └── src/
        └── services/
            └── srv/
                ├── getfeatures.srv
                └── setfeature.srv
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
│   │   │   ├── DeviceDetailsPanel.tsx
│   │   │   ├── DeviceSelectorPanel.tsx
│   │   │   ├── DeviceSubTaskbar.tsx
│   │   │   ├── DiscoveryPanel.tsx
│   │   │   ├── FeaturePanel.tsx
│   │   │   ├── ouster.tsx
│   │   │   ├── PresetsPanel.tsx
│   │   │   ├── record.tsx
│   │   │   ├── replay.tsx
│   │   │   ├── Settings.tsx
│   │   │   ├── Sidebar.tsx
│   │   │   ├── Taskbar.tsx
│   │   │   ├── WebTerminal.tsx
│   │   │   └── webviz.tsx
│   │   ├── handlers/       # Context handlers.
│   │   │   └── WindowSizeContext.tsx
│   │   ├── redux/          # Redux store and slices for state management.
│   │   │   ├── cameraSlice.ts
│   │   │   ├── dashboardSlice.ts
│   │   │   ├── devicesSlice.ts
│   │   │   ├── logSlice.ts
│   │   │   ├── ReduxProvider.tsx
│   │   │   ├── store.ts
│   │   │   └── thunks.ts
│   │   ├── utils/          # Utility functions.
│   │   │   └── apiConfig.ts
│   │   ├── favicon.ico
│   │   ├── globals.css
│   │   ├── layout.tsx
│   │   └── page.tsx
│   │   └── types.ts
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

*   **Camera Management (`camera_management`):** Manages camera-related operations, including automatic camera discovery, state management, creating, configuring, starting, stopping, and deleting camera nodes.

    | Endpoint | Method | Arguments| Description |
    | --- | --- | --- | --- |
    | `/discover_cameras` | `GET` | None | Discovers all available GigE cameras on the network.                     |
    | `/camera`                      | `POST` | `data` (Dict[str, str], expects "identifier") | Creates a new camera entry in the database and fetches its features.     |
    | `/camera/{camera_id}/start`    | `POST` | `camera_id` (int) | Starts the camera node for publishing.                                   |
    | `/camera/{camera_id}/stop`     | `POST` | `camera_id` (int) | Stops the camera node.                                                   |
    | `/cameras`                     | `GET`  | None | Retrieves all camera IDs from the database.                              |
    | `/camera/{camera_id}`          | `GET`  | `camera_id` (int)                       | Fetches camera details and its features from the database by ID.         |
    | `/camera/{camera_id}/notes`    | `POST` | `camera_id` (int), `data` (NotesData: `notes: str`) | Updates the user notes for a specific camera.                            |
    | `/camera/{camera_id}/publishing_preset` | `POST` | `camera_id` (int), `data` (PresetNameData: `preset_name: str`) | Updates the publishing preset for a specific camera.                     |
    | `/camera/{camera_id}`          | `DELETE` | `camera_id` (int)                       | Deletes a camera from the system.                                        |

* **Configuration Management (`configuration_management`):** Handles the loading and saving of system and device configurations.

    | Endpoint | Method | Arguments| Description |
    | --- | --- | --- | --- |
    | `/config/{config_name}`  | `GET`  | `config_name` (str)                     | Reads and returns the content of a specified YAML configuration file. |
    | `/config/{config_name}`  | `POST` | `config_name` (str), `config_data` (Dict) | Updates the content of a specified YAML configuration file. |

*   **Driver Management (`driver_management`):** Manages device drivers and their lifecycle.

    | Endpoint | Method | Arguments| Description |
    | --- | --- | --- | --- |
    | `/driver/{device_name}/start`  | `POST` | `device_name` (str), `protocol` (str, optional), `camera_id` (str, optional) | Starts the driver for a specified device and streams its logs.           |
    | `/driver/{device_name}/stop`   | `POST` | `device_name` (str), `protocol` (str, optional), `camera_id` (str, optional) | Stops the driver for a specified device.                                 |
    | `/driver/{device_name}/status` | `GET`  | `device_name` (str), `protocol` (str, optional), `camera_id` (str, optional) | Retrieves the current status of a specified device driver.               |

*   **Feature Control (`feature_control`):** Provides endpoints for controlling device features in real-time.

    | Endpoint | Method | Arguments| Description |
    | --- | --- | --- | --- |
    | `/set_feature/{camera_id}`     | `POST` | `camera_id` (str), `req` (SetFeatureRequest: `feature: str`, `value: str`) | Sets a feature for a specific camera.                                    |
    | `/get_features/{camera_id}`    | `GET`  | `camera_id` (str)                       | Retrieves all features for a specific camera.                            |
    | `/camera/{camera_id}/set_feature` | `POST` | `camera_id` (str), `data` (Dict[str, str], expects "feature", "value") | Sets a feature for a specific camera.                                    |
    | `/camera/{camera_id}/features` | `GET`  | `camera_id` (str)                       | Retrieves all features for a specific camera.                            |

*   **Logs (`logs`):** Manages system and device logs.

    | Endpoint | Method | Arguments| Description |
    | --- | --- | --- | --- |
    | `/logs`                  | `WS`   | None                                    | Establishes a WebSocket connection for streaming logs.   |

*   **Preset Management (`preset_management`):** Handles the creation, loading, and saving of device presets.

    | Endpoint | Method | Arguments| Description |
    | --- | --- | --- | --- |
    | `/camera/{camera_id}/save_preset`      | `POST` | `camera_id` (str), `preset_data` (PresetData: `name: str`) | Saves the current camera features as a new preset.                       |
    | `/presets/{device_identifier}`         | `POST` | `device_identifier` (str), `preset_data` (Dict[str, Any], expects "name", "configuration") | Creates a new preset for a given device.                                 |
    | `/presets/{device_identifier}`         | `GET`  | `device_identifier` (str)               | Retrieves all presets for a given device.                                |
    | `/presets/{device_identifier}/{preset_name}` | `GET`  | `device_identifier` (str), `preset_name` (str) | Retrieves a specific preset by device identifier and preset name.        |
    | `/presets/{device_identifier}/{preset_name}` | `PUT`  | `device_identifier` (str), `preset_name` (str), `preset_data` (Dict[str, Any], expects "configuration") | Updates an existing preset.                                              |
    | `/presets/{device_identifier}/{preset_name}` | `DELETE` | `device_identifier` (str), `preset_name` (str) | Deletes a preset.                                                        |
    | `/presets/load_from_yaml`              | `POST` | None                                    | Loads presets from the default 'presets.yaml' file in the config directory. |
    | `/presets/{device_identifier}/{preset_name}/apply` | `POST` | `device_identifier` (str), `preset_name` (str) | Applies a preset to a given device.                                      |

*   **Recording Management (`recording_management`):** Manages ROS bag recording and playback.

    | Endpoint | Method | Arguments| Description |
    | --- | --- | --- | --- |
    | `/recording/start`             | `POST` | None                                    | Starts a new ROS bag recording.                                          |
    | `/recording/stop`              | `POST` | None                                    | Stops the current ROS bag recording.                                     |
    | `/recording/status`            | `GET`  | None                                    | Retrieves the current status of the ROS bag recording process.          |
    | `/recordings`                  | `GET`  | None                                    | Lists all available ROS bag recordings.                                  |
    | `/recordings/{bag_name}`       | `GET`  | `bag_name` (str)                        | Downloads a specific ROS bag recording by its name.                      |
    | `/recordings/play/{bag_name}`  | `POST` | `bag_name` (str)                        | Plays a specific ROS bag recording.                                      |
    | `/recordings/stop_playback`   | `POST` | None                                    | Stops the current ROS bag playback.                                     |
    | `/recordings/convert`          | `POST` | `input_bag_name` (str), `output_format` (str, optional) | Converts a ROS bag file from one format to another.                     |

*   **Terminal (`terminal`):** Provides a web-based terminal for interacting with the system.

    | Endpoint | Method | Arguments| Description |
    | --- | --- | --- | --- |
    | `/terminal` | `WS`   | None | Establishes a WebSocket connection for interacting with the system terminal. |

Detailed API documentation, including request/response schemas and examples, can be accessed through the auto-generated OpenAPI/Swagger UI when the application is running.

---

## Database Schema

The application uses a PostgreSQL database. The schema is defined in the `init.sql` file located in the `ros_container` directory.

The main tables are:

*   **cameras:** Stores information about the connected cameras.
    *   `id` (SERIAL PRIMARY KEY)
    *   `identifier` (VARCHAR(255), UNIQUE)
    *   `name` (VARCHAR(255))
    *   `camera_ip` (VARCHAR(255))
    *   `notes` (TEXT)
    *   `publishing_preset` (VARCHAR(255))

*   **feature_groups:** Stores feature groups for cameras.
    *   `id` (SERIAL PRIMARY KEY)
    *   `camera_name` (VARCHAR(255) NOT NULL)

*   **features:** Stores individual features for cameras.
    *   `id` (SERIAL PRIMARY KEY)
    *   `group_id` (INTEGER REFERENCES feature_groups(id))
    *   `name` (VARCHAR(255) NOT NULL)
    *   `type` (VARCHAR(50))
    *   `value` (TEXT)
    *   `tooltip` (TEXT)
    *   `description` (TEXT)
    *   `min` (VARCHAR(50))
    *   `max` (VARCHAR(50))
    *   `options` (TEXT)
    *   `representation` (VARCHAR(50))
    *   `is_writable` (BOOLEAN DEFAULT FALSE)

*   **presets:** Stores the camera presets.
    *   `id` (SERIAL PRIMARY KEY)
    *   `camera_name` (VARCHAR(255) NOT NULL)
    *   `name` (VARCHAR(255))
    *   `configuration` (JSONB)

---
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

---

## Knowledge Base and Troubleshooting
#### Mounting Scripts and Volumes
- The `ros_container/api` folder is mounted into the `ros_container` at `/home/sourcingapi/ros2_ws/src/api`.
- The `start_services.sh` script is executed as the entrypoint for `ros_container` and now handles `rosbridge_server` setup and FastAPI launch.
- Example for mounting a script:
  ```yaml
  volumes:
    - ./create_user.sh:/home/create_user.sh
  ```
  This makes `create_user.sh` available at `/home/create_user.sh` inside the container.

---

## ToDo / Next Steps

---

## Changelog

### v0.1.2 Updates
*   **Automatic Camera Discovery and State:** Added automatic camera discovery and state management for seamless integration.
*   **Improved Naming Convention:** Enhanced naming convention by adding a better structure (`identifier` -> `hash`, `camera name` -> e.g., `xenics`, `camera ip` -> e.g., `localhost`).
*   **Resolved Build Issues:** Fully resolved all build issues related to permissions, directories, and dependencies.
*   **Cleaned Up Codebase:** Entire codebase has been cleaned up, removing unused scripts and code.
*   **Dependency Management:** Solved dependency issues by migrating to Poetry.
*   **Improved Build Time:** Optimized build processes to significantly reduce build time.
*   **Improved Build Size:** Reduced the overall build size for more efficient deployments.
*   **Improved Performance:** Enhanced application performance across various modules.
*   **Improved Dashboard:** The dashboard has been significantly improved by introducing Redux for state management.
*   **Fastapi router:** Added fastapi router to split the large script into groups and subfolders.