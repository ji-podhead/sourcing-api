import os
import subprocess
import signal
import logging
import asyncio
from fastapi import APIRouter, HTTPException
from typing import List, Dict, Optional
from utils.logs import LogConnectionManager
from devices.gig_e_driver import GigECameraNode
from .state import driver_processes, gige_camera_nodes, recording_process

logger = logging.getLogger(__name__)

router = APIRouter()
DEVICE_LAUNCH_SCRIPTS = {
    "camera": "imaging_source.py",
    "ouster": "ouster.py",
    "xenics": "xenics.py",
    # "new_device": "new_device_script.py" # Example for future extension
}

def is_process_running(proc: Optional[subprocess.Popen]) -> bool:
    """Checks if a subprocess.Popen object represents a running process."""
    return proc is not None and proc.poll() is None

@router.post("/driver/{device_name}/start")
async def start_driver(device_name: str, protocol: Optional[str] = None, camera_id: Optional[str] = None):
    """
    Starts the driver for a specified device and streams its logs.
    If the driver is already running, it returns an info message.
    """
    print(f"Starting driver for device: {device_name}, protocol: {protocol}, camera_id: {camera_id}")
    if device_name == "camera":
        if protocol == "gigE":
            if not camera_id:
                raise HTTPException(status_code=400, detail="camera_id is required for starting a GigE camera.")
            
            if camera_id in gige_camera_nodes:
                return {"status": "info", "message": f"GigE camera '{camera_id}' is already running."}
            
            try:
                node = GigECameraNode(camera_id=camera_id, ros_host='localhost')
                node.start()
                gige_camera_nodes[camera_id] = node
                logger.info(f"GigE camera '{camera_id}' started successfully via /driver/camera/start.")
                return {"status": "success", "message": f"GigE camera '{camera_id}' started."}
            except Exception as e:
                logger.error(f"Error starting GigE camera '{camera_id}': {e}")
                raise HTTPException(status_code=500, detail=f"Error starting GigE camera: {str(e)}")
        elif protocol == "GMSL2":
            raise HTTPException(status_code=501, detail=f"GMSL2 protocol is not yet supported for starting drivers.")
        else:
            raise HTTPException(status_code=400, detail="Protocol (gigE or GMSL2) and camera_id are required for starting a camera.")
    else: # Fallback to existing subprocess logic for other device_names
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
                                        cwd='/home/sourcingapi/', env=env)
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
                    logger.warning(f"{device_name.capitalize()} driver exited successfully immediately after start. This might be unexpected.")
                    return {"status": "success", "message": f"{device_name.capitalize()} driver started and exited cleanly."}

            logger.info(f"{device_name.capitalize()} driver started successfully with PID: {process.pid}")
            return {"status": "success", "message": f"{device_name.capitalize()} driver started."}
            
        except Exception as e:
            logger.error(f"Error starting {device_name} driver: {e}")
            if device_name in driver_processes and driver_processes[device_name].poll() is None:
                try:
                    os.killpg(os.getpgid(driver_processes[device_name].pid), signal.SIGINT)
                except Exception as cleanup_e:
                    logger.error(f"Error during cleanup of {device_name} process: {cleanup_e}")
            driver_processes.pop(device_name, None)
            raise HTTPException(status_code=500, detail=f"Error starting {device_name} driver: {str(e)}")

@router.post("/driver/{device_name}/stop")
async def stop_driver(device_name: str, protocol: Optional[str] = None, camera_id: Optional[str] = None):
    """
    Stops the driver for a specified device.
    If the driver is not running, it returns an info message.
    """
    if protocol == "gigE":
        if not camera_id:
            raise HTTPException(status_code=400, detail="camera_id is required for stopping a GigE camera.")
        
        node = gige_camera_nodes.pop(camera_id, None)
        if node:
            node.shutdown()
            logger.info(f"GigE camera '{camera_id}' stopped successfully.")
            return {"status": "success", "message": f"GigE camera '{camera_id}' stopped."}
        else:
            return {"status": "info", "message": f"GigE camera '{camera_id}' is not running."}
    elif protocol == "GMSL2":
        raise HTTPException(status_code=501, detail=f"GMSL2 protocol is not yet supported for stopping drivers.")
    else: # Fallback to existing subprocess logic
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

@router.get("/driver/camera/{camera_name}/status")
async def get_camera_driver_status(camera_name: str):
    """
    Retrieves the current status of a specific GigE camera driver.
    """
    node = gige_camera_nodes.get(camera_name)
    status = "running" if node and node.is_running else "stopped"
    logger.info(f"Status for GigE camera '{camera_name}': {status}")
    return {"device": camera_name, "status": status, "protocol": "gigE"}

@router.get("/driver/{device_name}/status")
async def get_driver_status(device_name: str):
    """
    Retrieves the current status of a specified device driver (non-camera devices).
    """
    if device_name not in DEVICE_LAUNCH_SCRIPTS:
        logger.warning(f"Status requested for unknown device: {device_name}")
        raise HTTPException(status_code=404, detail=f"Device '{device_name}' not found.")
        
    status = "running" if is_process_running(driver_processes.get(device_name)) else "stopped"
    logger.info(f"Status for {device_name} driver: {status}")
    return {"device": device_name, "status": status}

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
