import os
import subprocess
import signal
import logging
import asyncio
import sys
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.routing import APIRoute
from typing import List, Optional
from utils.logs import LogConnectionManager

logger = logging.getLogger(__name__)

routes = []
recording_process: Optional[subprocess.Popen] = None
ROS_SETUP_COMMAND = "source /opt/ros/humble/setup.bash"
CONFIG_DIR = "/home/sourcingapi/config" # Directory for configuration files
RECORDINGS_DIR = "/home/sourcingapi/data" # This should match the volume mount in docker-compose.yaml

def is_process_running(proc: Optional[subprocess.Popen]) -> bool:
    """Checks if a subprocess.Popen object represents a running process."""
    return proc is not None and proc.poll() is None

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

async def get_recording_status():
    """
    Retrieves the current status of the ROS bag recording process.
    Returns "running" if a recording is active, otherwise "stopped".
    """
    return {"status": "running" if is_process_running(recording_process) else "stopped"}

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

async def download_recording(bag_name: str):
    """
    Downloads a specific ROS bag recording by its name.
    """
    file_path = os.path.join(RECORDINGS_DIR, bag_name)
    if os.path.exists(file_path):
        return FileResponse(path=file_path, filename=bag_name, media_type="application/octet-stream")
    raise HTTPException(status_code=404, detail="Recording not found")

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

route1 = APIRoute("/recording/start", endpoint=start_recording, methods=["POST"])
route2 = APIRoute("/recording/stop", endpoint=stop_recording, methods=["POST"])
route3 = APIRoute("/recording/status", endpoint=get_recording_status, methods=["GET"])
route4 = APIRoute("/recordings", endpoint=list_recordings, methods=["GET"])
route5 = APIRoute("/recordings/{bag_name}", endpoint=download_recording, methods=["GET"])
route6 = APIRoute("/recordings/play/{bag_name}", endpoint=play_recording, methods=["POST"])
route7 = APIRoute("/recordings/stop_playback", endpoint=stop_playback, methods=["POST"])
route8 = APIRoute("/recordings/convert", endpoint=convert_bag_format, methods=["POST"])

routes.append(route1)
routes.append(route2)
routes.append(route3)
routes.append(route4)
routes.append(route5)
routes.append(route6)
routes.append(route7)
routes.append(route8)
