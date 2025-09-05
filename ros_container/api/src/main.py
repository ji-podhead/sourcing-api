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
from fastapi import Body
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
import yaml
from typing import List, Dict, Optional, Any
from datetime import datetime
from pydantic import BaseModel
# from ..utils.logs import LogConnectionManager
# from ..utils.camera_features import CameraFeatures, list_connected_cameras, initialize_camera_data
# Configure logging
from utils.logs import LogConnectionManager
from utils.gigE.camera_manager import initialize_camera_data, get_gigE_camera, get_all_gigE_cam_ids, get_db_connection # Import get_db_connection
from utils.db.db_utils import (
    create_preset,
    get_preset,
    get_presets_for_device,
    update_preset,
    delete_preset,
    update_camera_notes,
    update_camera_publishing_preset
)
from devices.gig_e_driver import GigECameraNode
from server.root import app

import server.camera_management
import server.driver_management
import server.feature_control
import server.preset_management
import server.recording_management
import server.configuration_management

# Add camera management routes
for route in server.camera_management.routes:
    app.add_api_route(route.path, route.endpoint, methods=route.methods)

# Add driver management routes
for route in server.driver_management.routes:
    app.add_api_route(route.path, route.endpoint, methods=route.methods)

# Add feature control routes
for route in server.feature_control.routes:
    app.add_api_route(route.path, route.endpoint, methods=route.methods)

# Add preset management routes
for route in server.preset_management.routes:
    app.add_api_route(route.path, route.endpoint, methods=route.methods)

# Add recording management routes
for route in server.recording_management.routes:
    app.add_api_route(route.path, route.endpoint, methods=route.methods)

# Add configuration management routes
for route in server.configuration_management.routes:
    app.add_api_route(route.path, route.endpoint, methods=route.methods)

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- CORS Configuration ---
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:5173"],  # Allow frontend origins
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"], # Explicitly allow common methods and OPTIONS for preflight requests
    allow_headers=["*"],
)

# TODO: Add endpoints for streaming (more complex, requires WebRTC or similar)x

# TODO: Implement GMSL2 camera initialization and feature handling in camera_manager.py

# TODO: Add frontend logic to fetch and display camera features based on their types

# --- Custom Feature Endpoints (ohne ROS .srv) ---
