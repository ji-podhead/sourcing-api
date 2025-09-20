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
from contextlib import asynccontextmanager
from fastapi import Body
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
import yaml
from typing import List, Dict, Optional, Any
from datetime import datetime
from pydantic import BaseModel
from devices.gig_e_driver import GigECameraNode
from routers.api import router as api_router

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Handles startup and shutdown events for the application.
    """
    logger.info("Application startup...")
    yield
    # Logic here runs on application shutdown
    from server.state import gige_camera_nodes
    # Heuristic to detect uvicorn reload: check if parent process is uvicorn
    try:
        parent_pid = os.getppid()
        parent_process = psutil.Process(parent_pid)
        if "uvicorn" in parent_process.name():
            logger.info("Detected uvicorn reload. Skipping camera process shutdown.")
            return # Skip the rest of the shutdown logic
    except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
        # If we can't get parent info, proceed with shutdown as normal.
        pass

    logger.info("Application shutting down. Cleaning up camera processes...")
    logger.info("Application shutting down. Cleaning up camera processes...")
    
    for camera_name, node_info in list(gige_camera_nodes.items()):
        logger.info(f"Stopping subprocess for camera '{camera_name}'.")
        try:
            process = node_info.get("process")
            command_queue = node_info.get("command_queue")
            
            if command_queue:
                command_queue.put("shutdown")
            
            if process and process.is_alive():
                process.join(timeout=5)
            
            if process and process.is_alive():
                logger.warning(f"Process for '{camera_name}' did not terminate gracefully. Forcefully terminating.")
                process.terminate()
        except Exception as e:
            logger.error(f"Error during shutdown of camera '{camera_name}': {e}")
            
    logger.info("All camera nodes shut down. Application shutdown complete.")


app = FastAPI(lifespan=lifespan)
websocket_origins = ["http://localhost:3000"]
websocket_paths = {
        "/terminal",
        "/logs",
        # Fügen Sie weitere WebSocket-Endpunkte hinzu
    }
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*","http://localhost:3000", "http://100.93.237.122:3000", "http://localhost:8000"], # Allow requests from the dashboard and direct API access
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(api_router)

@app.get("/")
async def read_root():
    """
    Root endpoint for the ROS Container FastAPI.
    Returns a simple message to confirm the API is running.
    
    """
    return {"message": "ROS Container FastAPI is running!"}

# TODO: Add endpoints for streaming (more complex, requires WebRTC or similar)x

# TODO: Implement GMSL2 camera initialization and feature handling in camera_manager.py

# TODO: Add frontend logic to fetch and display camera features based on their types

# --- Custom Feature Endpoints (ohne ROS .srv) ---


## OLD
# from ..utils.logs import LogConnectionManager
# from ..utils.camera_features import CameraFeatures, list_connected_cameras, initialize_camera_data
# Configure logging
# from utils.logs import LogConnectionManager
# from utils.gigE.camera_manager import initialize_camera_data, get_gigE_camera, get_all_gigE_cam_ids, get_db_connection # Import get_db_connection
# from utils.db.db_utils import (
#     create_preset,
#     get_preset,
#     get_presets_for_device,
#     update_preset,
#     delete_preset,
#     update_camera_notes,
#     update_camera_publishing_preset
# )
