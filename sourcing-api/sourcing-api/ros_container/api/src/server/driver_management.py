import os
import subprocess
import signal
import logging
import asyncio
from fastapi import APIRouter, HTTPException
from typing import List, Dict, Optional
from utils.logs import LogConnectionManager
from devices.gig_e_driver import run_camera_process
from .state import driver_processes, gige_camera_nodes, recording_process
from multiprocessing import Process, Queue

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
    Starts the driver for a specified device. For GigE cameras, it launches a dedicated process.
    """
    if device_name == "camera" and protocol == "gigE":
        if not camera_id:
            raise HTTPException(status_code=400, detail="camera_id is required for starting a GigE camera.")
        
        # Here, we use the camera_id string as the key, assuming it's the unique identifier.
        if camera_id in gige_camera_nodes:
            proc = gige_camera_nodes[camera_id].get('process')
            if proc and proc.is_alive():
                return {"status": "info", "message": f"GigE camera node for '{camera_id}' is already running."}
            else:
                logger.warning(f"Found a dead process entry for '{camera_id}'. Cleaning up.")
                gige_camera_nodes.pop(camera_id, None)
        
        try:
            logger.info(f"Starting subprocess for GigE camera '{camera_id}'.")
            command_queue = Queue()
            camera_id_int = int(camera_id)
            process = Process(target=run_camera_process, args=(camera_id_int, command_queue), daemon=True)
            process.start()
            
            gige_camera_nodes[camera_id] = {
                "process": process,
                "command_queue": command_queue
            }
            return {"status": "success", "message": f"GigE camera '{camera_id}' node started."}
        except ValueError:
            raise HTTPException(status_code=400, detail=f"Invalid camera_id format: '{camera_id}'. Must be an integer.")
        except Exception as e:
            logger.error(f"Error starting GigE camera '{camera_id}': {e}")
            raise HTTPException(status_code=500, detail=f"Error starting GigE camera: {str(e)}")

    elif protocol == "GMSL2":
        raise HTTPException(status_code=501, detail=f"GMSL2 protocol is not yet supported for starting drivers.")
    
    # Fallback to existing subprocess logic for other device_names
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
        
        asyncio.create_task(stream_subprocess_output(process, logger, device_name))
        asyncio.create_task(stream_subprocess_error(process, logger, device_name))
        
        await asyncio.sleep(2)

        if process.poll() is not None:
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
    """
    if protocol == "gigE":
        if not camera_id:
            raise HTTPException(status_code=400, detail="camera_id is required for stopping a GigE camera.")
        
        node_info = gige_camera_nodes.pop(camera_id, None)
        if node_info:
            logger.info(f"Stopping subprocess for GigE camera '{camera_id}'.")
            node_info["command_queue"].put("shutdown")
            node_info["process"].join(timeout=5)
            
            if node_info["process"].is_alive():
                logger.warning(f"Process for '{camera_id}' did not terminate gracefully. Terminating.")
                node_info["process"].terminate()
            
            return {"status": "success", "message": f"GigE camera '{camera_id}' stopped."}
        else:
            return {"status": "info", "message": f"GigE camera '{camera_id}' is not running."}

    elif protocol == "GMSL2":
        raise HTTPException(status_code=501, detail=f"GMSL2 protocol is not yet supported for stopping drivers.")
    
    # Fallback to existing subprocess logic for other device_names
    if device_name not in DEVICE_LAUNCH_SCRIPTS:
        raise HTTPException(status_code=404, detail=f"Device '{device_name}' not found.")

    process = driver_processes.get(device_name)
    if not is_process_running(process):
        return {"status": "info", "message": f"{device_name.capitalize()} driver is not running."}

    try:
        logger.info(f"Attempting to stop {device_name} driver with PID: {process.pid}")
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=5)
        driver_processes.pop(device_name, None)
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
    # This endpoint uses camera_name as the key. The start/stop endpoints for gigE use camera_id.
    # This is inconsistent. I will assume the key in gige_camera_nodes is the camera_id string.
    # The user might need to call this endpoint with the ID, not the name.
    node_info = gige_camera_nodes.get(camera_name)
    is_running = node_info and node_info.get('process') and node_info['process'].is_alive()
    status = "running" if is_running else "stopped"
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
