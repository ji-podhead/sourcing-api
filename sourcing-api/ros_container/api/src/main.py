import os
import subprocess
import signal
import psutil # For checking if a process is running
import sys # For running external python scripts
import logging # Import logging module
import time # For adding delays
import asyncio # For running subprocess asynchronously
import pty # For pseudo-terminal
import select # For non-blocking I/O with pty

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
import yaml
from typing import List, Dict, Optional, Any
from datetime import datetime
# from ..utils.logs import LogConnectionManager
# from ..utils.camera_features import CameraFeatures, list_connected_cameras, initialize_camera_data
# Configure logging
from utils.logs import LogConnectionManager
from utils.gigE.camera_manager import initialize_camera_data, get_gigE_camera, get_all_gigE_cam_ids, get_db_connection # Import get_db_connection
from utils.db.db_utils import ( # Import preset functions
    create_preset,
    get_preset,
    get_presets_for_device,
    update_preset,
    delete_preset
)

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Initialize FastAPI app FIRST
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*","http://localhost:3000", "http://100.93.237.122:3000", "http://localhost:8000"], # Allow requests from the dashboard and direct API access
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ROS_SETUP_COMMAND = "source /opt/ros/humble/setup.bash"
RECORDINGS_DIR = "/home/developer/data" # This should match the volume mount in docker-compose.yaml
CONFIG_DIR = "/home/developer/config" # Directory for configuration files
log_manager = LogConnectionManager(logger)

@app.websocket("/logs")
async def websocket_logs(websocket: WebSocket):
    logger.info("WebSocket connection established for logs.")
    await log_manager.connect(websocket)
    try:
        logger.info("WebSocket connection for logs is now active.")
        # Keep the connection open and wait for disconnect.
        # The log streaming is handled by the stream_logs task.
        while True:
            await asyncio.sleep(1) # Yield control to the event loop
    except WebSocketDisconnect:
        log_manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"Error in websocket_logs handler: {e}")
        log_manager.disconnect(websocket) # Ensure disconnect is called on error

@app.websocket("/terminal")
async def websocket_terminal(websocket: WebSocket):
    await websocket.accept()
    
    # Create a pseudo-terminal
    master_fd, slave_fd = pty.openpty()
    
    # Start a shell in the pseudo-terminal
    # Using bash for better compatibility and to source ROS setup
    shell_command = ["/bin/bash", "-l"] # -l makes it a login shell, sourcing .bashrc
    process = subprocess.Popen(
        shell_command,
        stdin=slave_fd,
        stdout=slave_fd,
        stderr=slave_fd,
        preexec_fn=os.setsid, # Detach from controlling terminal
        env=os.environ.copy() # Inherit current environment
    )
    
    os.close(slave_fd) # Close slave_fd in the parent process

    logger.info(f"Terminal WebSocket connected. Shell process PID: {process.pid}")

    try:
        while True:
            # Read from WebSocket and write to PTY master
            try:
                data = await websocket.receive_text()
                os.write(master_fd, data.encode())
            except WebSocketDisconnect:
                logger.info("Terminal WebSocket disconnected.")
                break
            except Exception as e:
                logger.error(f"Error receiving from WebSocket or writing to PTY: {e}")
                break

            # Read from PTY master and write to WebSocket
            try:
                # Use a non-blocking read with a timeout
                # This is a simple way to prevent blocking indefinitely.
                # For more robust non-blocking I/O, consider asyncio.StreamReader
                # or a separate thread for PTY reading.
                r, _, _ = select.select([master_fd], [], [], 0.01) # 10ms timeout
                if r:
                    output = os.read(master_fd, 4096).decode(errors='ignore')
                    await websocket.send_text(output)
            except Exception as e:
                logger.error(f"Error reading from PTY or sending to WebSocket: {e}")
                break
    finally:
        # Clean up
        process.terminate()
        process.wait()
        os.close(master_fd)
        logger.info(f"Terminal shell process {process.pid} terminated and PTY closed.")

# Register the startup event
@app.on_event("startup")
async def startup_event():
    logger.info("FastAPI application starting up...")

    # Preload cameras from DB on startup
    await get_all_cams("gigE")
    logger.info("Startup event finished.")

@app.get("/get_cam/{protocol}/{camera_identifier}")
async def get_cam(protocoll: str, camera_identifier:str):
    if(protocoll == "gigE"):
        data=get_gigE_camera(camera_identifier)
        if(data is None):
            raise HTTPException(status_code=404, detail=f"Camera '{camera_identifier}' not found.")
        else:
            return data
    else:
        raise HTTPException(status_code=404, detail=f"Protocol '{protocoll}' not supported.")

@app.get("/get_all_cams/{protocol}")
async def get_all_cams(protocol: str):
    # Strip any leading/trailing single quotes from the protocol string
    protocol = protocol.strip("'")
    if(protocol == "gigE"):
        stored_cameras = await get_all_gigE_cam_ids()
        logger.info(f"Stored cameras found in DB: {[cam for cam in stored_cameras]}")
        all_camera_data = []
        for cam_id in stored_cameras:
            logger.info(f"Updating camera data for gigE camera with ID: {cam_id}")
            await update_cam("gigE", cam_id) # Ensure features are up-to-date
            
            camera_details = await get_gigE_camera(cam_id)
            if camera_details:
                # Augment camera_details with frontend-expected fields
                augmented_camera_data = {
                    **camera_details,
                    "name": f"Camera: {camera_details['identifier']}",
                    "path": "/imagingsource",  # Generic path for dynamic cameras
                    "apiEndpoint": "/driver/camera",  # Generic API endpoint for dynamic cameras
                    "fileName": f"camera_{camera_details['id']}.yaml",  # Placeholder for config file name
                }
                all_camera_data.append(augmented_camera_data)
        return all_camera_data
    else:
        raise HTTPException(status_code=404, detail=f"Protocol '{protocol}' not supported.")
        return None

@app.post("/update_cam/{protocol}/{camera_identifier}")
async def update_cam(protocoll: str, camera_identifier:str):
    if(protocoll == "gigE"):
        await initialize_camera_data(camera_identifier)

@app.post("/create_camera")
async def create_camera(camera_data: Dict[str, str]):
    """
    Creates a new camera based on provided ID and protocol.
    """
    logger.info(f"Received camera data for creation: {camera_data}") # Log received data
    protocol = camera_data.get("protocol")
    camera_id = camera_data.get("id") # Frontend sends 'id'

    if not protocol or not camera_id:
        raise HTTPException(status_code=400, detail="Protocol and camera ID are required.")

    logger.info(f"Received request to create camera: Protocol={protocol}, ID={camera_id}")
    try:
        if protocol == "gigE":
            # Basic validation for IP address format (can be improved)
            if not is_valid_ip(camera_id):
                raise HTTPException(status_code=400, detail=f"Invalid IP address format for gigE camera: {camera_id}")
            logger.info(f"Validated IP address for gigE camera: {camera_id}")
            existing_camera = await get_gigE_camera(camera_id)
            logger.info(f"Existing camera check for ID '{camera_id}': {'Found' if existing_camera else 'Not Found'}")
            if(existing_camera):
                logger.info(f"gigE camera with ID '{camera_id}' already exists. Proceeding to update its features.")
                await update_cam("gigE", camera_id)
                return {"status": "info", "message": f"gigE camera '{camera_id}' already exists. Features updated.", "camera_data": existing_camera}
            else:
                logger.info(f"gigE camera with ID '{camera_id}' does not exist. Proceeding to add and initialize.")            
                await initialize_camera_data(camera_id) 
                updated_camera_data = await get_gigE_camera(camera_id)             
                if existing_camera:
                    return {"status": "success", "message": f"gigE camera '{camera_id}' updated successfully.", "camera_data": updated_camera_data}
                else:
                    return {"status": "success", "message": f"gigE camera '{camera_id}' added and initialized successfully.", "camera_data": updated_camera_data}

        elif protocol == "GMSL2":
            # Placeholder for GMSL2 camera handling
            # This would require specific logic for GMSL2 cameras, potentially using different drivers or initialization functions.
            # For now, we'll return an error indicating it's not yet supported.
            logger.warning(f"GMSL2 protocol is not yet supported for camera creation.")
            raise HTTPException(status_code=501, detail=f"GMSL2 protocol is not yet supported.")
        else:
            raise HTTPException(status_code=400, detail=f"Protocol '{protocol}' not supported for creating cameras.")
    except HTTPException as e:
        raise e
    except Exception as e:
        logger.error(f"Error creating/updating camera '{camera_id}' with protocol '{protocol}': {e}")
        raise HTTPException(status_code=500, detail=f"Error creating/updating camera: {str(e)}")

# Helper function for IP validation (can be expanded for more robust checks)
def is_valid_ip(ip_address: str) -> bool:
    import ipaddress
    try:
        ipaddress.ip_address(ip_address)
        return True
    except ValueError:
        return False



# --- REFACTOR START: Generic Driver Endpoints ---
# This comment is already in the file. I will insert my code before it.

# Helper function to read and log subprocess output
async def stream_subprocess_output(process: subprocess.Popen, logger: logging.Logger, device_name: str):
    loop = asyncio.get_event_loop()
    try:
        while True:
            # Check if the process is still running
            if process.poll() is not None:
                break
            
            # Read a line from stdout asynchronously
            if process.stdout.closed:
                break
            
            line = await loop.run_in_executor(None, process.stdout.readline)
            if not line:
                break
            
            logger.info(f"[{device_name.capitalize()} STDOUT] {line.strip()}")
    except Exception as e:
        logger.error(f"Error reading stdout for {device_name}: {e}")

# Helper function to read and log subprocess error
async def stream_subprocess_error(process: subprocess.Popen, logger: logging.Logger, device_name: str):
    loop = asyncio.get_event_loop()
    try:
        while True:
            # Check if the process is still running
            if process.poll() is not None:
                break
            
            # Read a line from stderr asynchronously
            if process.stderr.closed:
                break
            
            line = await loop.run_in_executor(None, process.stderr.readline)
            if not line:
                break
            
            logger.error(f"[{device_name.capitalize()} STDERR] {line.strip()}")
    except Exception as e:
        logger.error(f"Error reading stderr for {device_name}: {e}")

# --- REFACTOR START: Generic Driver Endpoints ---
driver_processes: Dict[str, subprocess.Popen] = {}
recording_process: Optional[subprocess.Popen] = None

# Central configuration for device launch scripts.
# To add a new device, simply add an entry here.
DEVICE_LAUNCH_SCRIPTS = {
    "camera": "imaging_source.py",
    "ouster": "ouster.py",
    "xenics": "xenics.py",
    # "new_device": "new_device_script.py" # Example for future extension
}
# --- REFACTOR END ---


def is_process_running(proc: Optional[subprocess.Popen]) -> bool:
    """Checks if a subprocess.Popen object represents a running process."""
    return proc is not None and proc.poll() is None

@app.get("/")
async def read_root():
    """
    Root endpoint for the ROS Container FastAPI.
    Returns a simple message to confirm the API is running.
    """
    return {"message": "ROS Container FastAPI is running!"}

# --- REFACTOR START: Generic Driver Endpoints ---

@app.post("/driver/{device_name}/start")
async def start_driver(device_name: str):
    """
    Starts the driver for a specified device and streams its logs.
    If the driver is already running, it returns an info message.
    """
    if device_name not in DEVICE_LAUNCH_SCRIPTS:
        raise HTTPException(status_code=404, detail=f"Device '{device_name}' not found.")

    if is_process_running(driver_processes.get(device_name)):
        return {"status": "info", "message": f"{device_name.capitalize()} driver is already running."}
    
    try:
        script_name = DEVICE_LAUNCH_SCRIPTS[device_name]
        launch_file_path = os.path.join(os.path.dirname(__file__), '..', 'devices', script_name)
        
        command = f"python3 {launch_file_path}"
        
        logger.info(f"Attempting to start {device_name} driver with command: {command}")
        env = os.environ.copy()
        env['ROS_PACKAGE_PATH'] = '/root/ros2_ws:' + env.get('ROS_PACKAGE_PATH', '')
        
        process = subprocess.Popen(command.split(), shell=False, preexec_fn=os.setsid,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                     cwd='/home/developer/', env=env)
        driver_processes[device_name] = process
        
        # Start tasks to stream stdout and stderr asynchronously
        asyncio.create_task(stream_subprocess_output(process, logger, device_name))
        asyncio.create_task(stream_subprocess_error(process, logger, device_name))
        
        # Give the process a moment to start and potentially fail.
        # We rely on the streaming tasks to report errors if they occur.
        await asyncio.sleep(2) # Increased delay to allow process to initialize and potentially log errors

        # Check if the process terminated unexpectedly shortly after starting
        if process.poll() is not None:
            # If it terminated, the streaming tasks would have logged the error.
            # We just need to signal the failure.
            if process.returncode != 0:
                 raise HTTPException(status_code=500, detail=f"{device_name.capitalize()} driver failed to start. Check logs for details.")
            else:
                # Process exited with code 0, which is unexpected for a long-running driver.
                logger.warning(f"{device_name.capitalize()} driver exited successfully immediately after start. This might be unexpected.")
                return {"status": "success", "message": f"{device_name.capitalize()} driver started and exited cleanly."}

        # If we reach here, the process is running.
        logger.info(f"{device_name.capitalize()} driver started successfully with PID: {process.pid}")
        return {"status": "success", "message": f"{device_name.capitalize()} driver started."}
        
    except Exception as e:
        logger.error(f"Error starting {device_name} driver: {e}")
        # Ensure the process is cleaned up if an exception occurs before it's fully managed
        if device_name in driver_processes and driver_processes[device_name].poll() is None:
            try:
                os.killpg(os.getpgid(driver_processes[device_name].pid), signal.SIGINT)
            except Exception as cleanup_e:
                logger.error(f"Error during cleanup of {device_name} process: {cleanup_e}")
        driver_processes.pop(device_name, None)
        raise HTTPException(status_code=500, detail=f"Error starting {device_name} driver: {str(e)}")

@app.post("/driver/{device_name}/stop")
async def stop_driver(device_name: str):
    """
    Stops the driver for a specified device.
    If the driver is not running, it returns an info message.
    """
    if device_name not in DEVICE_LAUNCH_SCRIPTS:
        raise HTTPException(status_code=404, detail=f"Device '{device_name}' not found.")

    process = driver_processes.get(device_name)
    if not is_process_running(process):
        return {"status": "info", "message": f"{device_name.capitalize()} driver is not running."}

    try:
        logger.info(f"Attempting to stop {device_name} driver with PID: {process.pid}")
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=5)  # Wait for process to terminate
        driver_processes.pop(device_name, None)  # Remove from tracking
        logger.info(f"{device_name.capitalize()} driver stopped successfully.")
        return {"status": "success", "message": f"{device_name.capitalize()} driver stopped."}
    except Exception as e:
        logger.error(f"Failed to stop {device_name} driver (PID: {process.pid}): {e}")
        raise HTTPException(status_code=500, detail=f"Failed to stop {device_name} driver: {e}")

@app.get("/driver/{device_name}/status")
async def get_driver_status(device_name: str):
    """
    Retrieves the current status of a specified device driver.
    Returns "running" if the driver process is active, otherwise "stopped".
    """
    if device_name not in DEVICE_LAUNCH_SCRIPTS:
        logger.warning(f"Status requested for unknown device: {device_name}")
        raise HTTPException(status_code=404, detail=f"Device '{device_name}' not found.")
        
    status = "running" if is_process_running(driver_processes.get(device_name)) else "stopped"
    logger.info(f"Status for {device_name} driver: {status}")
    return {"device": device_name, "status": status}

# --- REFACTOR END ---

# --- Preset Management Endpoints ---

@app.post("/presets/{device_identifier}")
async def create_preset_endpoint(device_identifier: str, preset_data: Dict[str, Any]):
    """Creates a new preset for a given device."""
    preset_name = preset_data.get("name")
    configuration = preset_data.get("configuration")

    if not preset_name or not configuration:
        raise HTTPException(status_code=400, detail="Preset name and configuration are required.")

    if create_preset(device_identifier, preset_name, configuration):
        return {"status": "success", "message": f"Preset '{preset_name}' created for device '{device_identifier}'."}
    else:
        raise HTTPException(status_code=500, detail=f"Failed to create preset '{preset_name}' for device '{device_identifier}'.")

@app.get("/presets/{device_identifier}")
async def get_presets_for_device_endpoint(device_identifier: str):
    """Retrieves all presets for a given device."""
    presets = await get_presets_for_device(device_identifier)
    if presets is not None:
        return {"device_identifier": device_identifier, "presets": presets}
    else:
        # This case should ideally not happen if get_presets_for_device handles errors internally
        raise HTTPException(status_code=500, detail=f"Failed to retrieve presets for device '{device_identifier}'.")

@app.get("/presets/{device_identifier}/{preset_name}")
async def get_preset_endpoint(device_identifier: str, preset_name: str):
    """Retrieves a specific preset by device identifier and preset name."""
    preset_config = await get_preset(device_identifier, preset_name)
    if preset_config:
        return {"device_identifier": device_identifier, "preset_name": preset_name, "configuration": preset_config}
    else:
        raise HTTPException(status_code=404, detail=f"Preset '{preset_name}' not found for device '{device_identifier}'.")

@app.put("/presets/{device_identifier}/{preset_name}")
async def update_preset_endpoint(device_identifier: str, preset_name: str, preset_data: Dict[str, Any]):
    """Updates an existing preset."""
    new_configuration = preset_data.get("configuration")
    if not new_configuration:
        raise HTTPException(status_code=400, detail="New configuration is required for update.")

    if update_preset(device_identifier, preset_name, new_configuration):
        return {"status": "success", "message": f"Preset '{preset_name}' updated for device '{device_identifier}'."}
    else:
        # This could mean the preset was not found or an error occurred
        raise HTTPException(status_code=404, detail=f"Preset '{preset_name}' not found or failed to update for device '{device_identifier}'.")

@app.delete("/presets/{device_identifier}/{preset_name}")
async def delete_preset_endpoint(device_identifier: str, preset_name: str):
    """Deletes a preset."""
    if delete_preset(device_identifier, preset_name):
        return {"status": "success", "message": f"Preset '{preset_name}' deleted for device '{device_identifier}'."}
    else:
        raise HTTPException(status_code=404, detail=f"Preset '{preset_name}' not found for device '{device_identifier}'.")

# --- END Preset Management Endpoints ---


@app.post("/recording/start")
async def start_recording():
    """
    Starts a new ROS bag recording.
    A unique bag file name is generated. Drivers can be
    launched by the recording script if they are not already running.
    """
    global recording_process
    if is_process_running(recording_process):
        return {"status": "info", "message": "Recording is already in progress."}
    try:
        # Generate a unique bag file name
        bag_name_script_path = os.path.join(os.path.dirname(__file__), '..', 'utils', 'create_rosbag_name.py')
        bag_name_process = subprocess.run([sys.executable, bag_name_script_path], capture_output=True, text=True, check=True)
        bag_name = bag_name_process.stdout.strip()

        # --- REFACTOR: Update check for running drivers ---
        # Determine if drivers need to be launched by the recording script
        launch_camera = "false" if is_process_running(driver_processes.get("camera")) else "true"
        launch_ouster = "false" if is_process_running(driver_processes.get("ouster")) else "true"

        # Construct the launch command with arguments
        launch_file_path = os.path.join(os.path.dirname(__file__), '..', 'devices', 'record.py')
        command = (
            f"{ROS_SETUP_COMMAND} && python3 {launch_file_path}"
            f" destination:={RECORDINGS_DIR}/{bag_name}"
            f" launch_camera_driver:={launch_camera}"
            f" launch_ouster_driver:={launch_ouster}"
        )
        recording_process = subprocess.Popen(command, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        return {"status": "success", "message": f"Recording started as {bag_name}."}
    except subprocess.CalledProcessError as e:
        raise HTTPException(status_code=500, detail=f"Failed to generate bag name: {e.stderr}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/recording/stop")
async def stop_recording():
    """
    Stops the current ROS bag recording.
    If no recording is in progress, it returns an info message.
    """
    global recording_process
    if not is_process_running(recording_process):
        return {"status": "info", "message": "No recording in progress."}
    try:
        os.killpg(os.getpgid(recording_process.pid), signal.SIGINT)
        recording_process.wait(timeout=5)
        recording_process = None
        return {"status": "success", "message": "Recording stopped."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to stop recording: {e}")

@app.get("/recording/status")
async def get_recording_status():
    """
    Retrieves the current status of the ROS bag recording process.
    Returns "running" if a recording is active, otherwise "stopped".
    """
    return {"status": "running" if is_process_running(recording_process) else "stopped"}

@app.get("/recordings")
async def list_recordings() -> List[str]:
    """
    Lists all available ROS bag recordings in the designated recordings directory.
    Includes .bag, .db3, and .mcap files.
    """
    try:
        if not os.path.exists(RECORDINGS_DIR):
            return []
        # Include .bag, .db3, and .mcap files for ROS2 bags
        bags = [f for f in os.listdir(RECORDINGS_DIR) if f.endswith(".bag") or f.endswith(".db3") or f.endswith(".mcap")]
        return bags
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/recordings/{bag_name}")
async def download_recording(bag_name: str):
    """
    Downloads a specific ROS bag recording by its name.
    """
    file_path = os.path.join(RECORDINGS_DIR, bag_name)
    if os.path.exists(file_path):
        return FileResponse(path=file_path, filename=bag_name, media_type="application/octet-stream")
    raise HTTPException(status_code=404, detail="Recording not found")

@app.post("/recordings/play/{bag_name}")
async def play_recording(bag_name: str):
    """
    Plays a specific ROS bag recording.
    Only one playback or recording process can be active at a time.
    """
    global recording_process # Reusing recording_process for playback
    if is_process_running(recording_process):
        return {"status": "info", "message": "Another recording or playback is already in progress."}
    try:
        file_path = os.path.join(RECORDINGS_DIR, bag_name)
        if not os.path.exists(file_path):
            raise HTTPException(status_code=404, detail="Recording not found")

        command = f"{ROS_SETUP_COMMAND} && ros2 bag play {file_path}"
        recording_process = subprocess.Popen(command, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
        return {"status": "success", "message": f"Playing recording {bag_name}."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to start playback: {e}")

@app.post("/recordings/stop_playback")
async def stop_playback():
    """
    Stops the current ROS bag playback.
    """
    global recording_process
    if not is_process_running(recording_process):
        return {"status": "info", "message": "No playback in progress."}
    try:
        os.killpg(os.getpgid(recording_process.pid), signal.SIGINT)
        recording_process.wait(timeout=5)
        recording_process = None
        return {"status": "success", "message": "Playback stopped."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to stop playback: {e}")

@app.post("/recordings/convert")
async def convert_bag_format(input_bag_name: str, output_format: str = "sqlite3"):
    """
    Converts a ROS bag file from one format to another.
    """
    input_path = os.path.join(RECORDINGS_DIR, input_bag_name)
    if not os.path.exists(input_path):
        raise HTTPException(status_code=404, detail=f"Input bag '{input_bag_name}' not found.")

    input_storage_id = "mcap" if input_bag_name.endswith(".mcap") else "sqlite3"
    output_bag_name = os.path.splitext(input_bag_name)[0] + "." + ("db3" if output_format == "sqlite3" else output_format)
    output_path = os.path.join(RECORDINGS_DIR, output_bag_name)

    try:
        command = (
            f"{ROS_SETUP_COMMAND} && ros2 bag convert"
            f" -i {input_path}"
            f" -o {output_path}"
            f" -s {input_storage_id}"
            f" --output-storage {output_format}"
        )
        result = subprocess.run(command, shell=True, executable="/bin/bash", capture_output=True, text=True, check=True)
        return {"status": "success", "message": f"Conversion of {input_bag_name} to {output_bag_name} successful.", "output": result.stdout, "error": result.stderr}
    except subprocess.CalledProcessError as e:
        raise HTTPException(status_code=500, detail=f"Bag conversion failed: {e.stderr}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"An unexpected error occurred during conversion: {str(e)}")

@app.get("/config/{config_name}")
async def get_config(config_name: str):
    """
    Reads and returns the content of a specified YAML configuration file.
    """
    file_path = os.path.join(CONFIG_DIR, config_name)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail=f"Configuration file '{config_name}' not found.")
    try:
        with open(file_path, 'r') as f:
            config_content = yaml.safe_load(f)
        return config_content
    except yaml.YAMLError as e:
        raise HTTPException(status_code=500, detail=f"Error parsing YAML file: {e}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to read configuration file: {e}")

@app.post("/config/{config_name}")
async def update_config(config_name: str, config_data: Dict):
    """
    Updates the content of a specified YAML configuration file.
    """
    file_path = os.path.join(CONFIG_DIR, config_name)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail=f"Configuration file '{config_name}' not found.")
    try:
        with open(file_path, 'w') as f:
            yaml.dump(config_data, f, sort_keys=False) # Preserve order
        return {"status": "success", "message": f"Configuration file '{config_name}' updated successfully."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to write configuration file: {e}")

# TODO: Add endpoints for streaming (more complex, requires WebRTC or similar)x

# TODO: Implement GMSL2 camera initialization and feature handling in camera_manager.py

# TODO: Add frontend logic to fetch and display camera features based on their types
