import gi
gi.require_version('Aravis', '0.8')
import logging
from typing import Dict
from gi.repository import Aravis
from utils.gigE.camera_manager import initialize_camera_data, get_cam_identifiers
from fastapi import APIRouter, HTTPException
from utils.db.db_utils import (
    get_all_gigE_cam_ids,
    get_gigE_camera,
    create_gigE_camera,
    update_camera_notes,
        update_camera_publishing_preset,
    find_camera_by_identifier,
    get_camera_name_by_id,
    delete_camera_from_db
)
from devices.gig_e_driver import GigECameraNode
from pydantic import BaseModel
from .state import gige_camera_nodes
from utils.gigE.camera_manager import update_camera_features

logger = logging.getLogger(__name__)

router = APIRouter()

@router.post("/camera/{camera}/update_features")
async def update_camera_features_endpoint(camera: int | str):
    """Updates the features for a given camera."""
    identifiers=await get_cam_identifiers(camera)
    camera_id=identifiers["id"] if identifiers["id"] is not None else camera
    await update_camera_features(camera_id)
    return {"status": "success", "message": f"Features updated for camera '{camera_id}'."}

CONFIG_DIR = "/home/sourcingapi/config" # Directory for configuration files

  
@router.get("/camera_ip/{camera}")
async def get_camera_ip(camera: str | int) -> str:
    try:
        identifiers=await get_cam_identifiers(camera)
        id=identifiers["id"] if identifiers["id"] is not None else camera
        Aravis.update_device_list()
        n_devices = Aravis.get_n_devices()
        if n_devices == 0:
            logger.info("No devices found on the network.")
            return []
        discovered_cameras = []
        logger.info(f"Number of devices found: {n_devices}")
        for i in range(n_devices):
            device_id = Aravis.get_device_id(i)
            if device_id == id:
                ip = Aravis.get_device_address(i)
                break

        if not ip:
            raise HTTPException(status_code=404, detail=f"Camera with ID '{camera}' not found on the network.")
        ip_address = Aravis.get_device_address(0)
        return ip_address
    except Exception as e:
        logger.error(f"Error getting IP address for camera '{camera}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/discover_cameras")
async def discover_cameras():
    """Discovers all available GigE cameras on the network."""
    try:
        Aravis.update_device_list()
        n_devices = Aravis.get_n_devices()
        if n_devices == 0:
            return []

        discovered_cameras = []
        logger.info(f"Number of devices found: {n_devices}")
        for i in range(n_devices):
            e = None
            print(f"Processing device index {i}")
            try:
                device_id = Aravis.get_device_id(i)
                
                cam = Aravis.Camera.new(device_id)
                if not cam:
                    continue

                name = cam.get_model_name()
                print(f"Camera Name: {name}")
                print(f"Device ID: {device_id}")
                ip_address = "N/A"
                existing_id = None
                gv_device = cam.props.device
                try:
                    ip_address = Aravis.get_device_address(i)
                    
            
                    print(f"Camera IP: {ip_address}")
                except Exception as e:
                    logger.error(f"Error getting IP address for device {device_id}: {e}")
                    ip_address = "N/A"

                existing_id = await find_camera_by_identifier(device_id)

                response_device_id = existing_id if existing_id is not None else "Not in DB"

                discovered_cameras.append({
                    "name": device_id,
                    "ip_address": ip_address,
                    "device_id": response_device_id,
                    "is_in_db": existing_id is not None,
                    "db_id": existing_id
                })
            except Exception as e:
                logger.error(f"Error processing device index {i}: {e}")
                continue
        return discovered_cameras
    except Exception as e:
        logger.error(f"Error during camera discovery: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/camera")
async def create_camera(data: Dict[str, str]):
    """Creates a new camera entry in the database and fetches its features."""
    identifier = data.get("identifier")
    if not identifier:
        raise HTTPException(status_code=400, detail="Identifier is required.")
    try:
            cam = Aravis.Camera.new(identifier)
            if not cam:
                raise HTTPException(status_code=404, detail=f"Camera with identifier '{identifier}' not found on the network.")

            camera_name = identifier
            camera_ip = ""
            try:
                camera_ip = await get_camera_ip(camera_name)
                logger.info(f"Camera IP address for device {identifier}: {camera_ip}")
            except Exception as e:
                logger.error(f"Error getting IP address for device {identifier}: {e}")
                camera_ip = ""
            print(f"Camera IP: {camera_ip}")

            existing_id = await find_camera_by_identifier(camera_name)
            if existing_id is not "Not in DB" and existing_id is not None:
                return await get_gigE_camera(existing_id)
            else:
                # Initialize camera data to get features
                new_camera_id = await initialize_camera_data(camera_name, camera_ip)
            if not new_camera_id:
                raise HTTPException(status_code=500, detail="Failed to initialize camera and fetch features.")

            # After initialization, fetch and return the complete camera data
            new_camera_data = await get_gigE_camera(new_camera_id)
            if new_camera_data is None:
                raise HTTPException(status_code=404, detail=f"Failed to retrieve newly created camera with ID {new_camera_id}.")
            return new_camera_data
    except Exception as e:
        logger.error(f"Error creating camera with identifier '{identifier}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/camera/{camera}/start")
async def start_camera(camera: int | str):
    """Starts the camera node for publishing."""
    identifiers=await get_cam_identifiers(camera)
    camera_id=identifiers["id"] if identifiers["id"] is not None else camera
    if(camera_id is None and isinstance(camera,str)):
        return {"status": "error", "message": f"Camera '{camera}' not found in database."}
    camera_name = get_camera_name_by_id(camera_id)
    if not camera_name:
        raise HTTPException(status_code=404, detail=f"Camera with ID {camera_id} not found.")

    if camera_name in gige_camera_nodes:
        return {"status": "info", "message": "Camera node is already running."}

    node = GigECameraNode(camera_id=camera_name, ros_host='localhost')
    node.start()
    gige_camera_nodes[camera_name] = node

    return {"status": "success", "message": f"Camera '{camera_name}' node started successfully."}

@router.post("/camera/{camera}/stop")
async def stop_camera(camera: int | str ):
    """Stops the camera node."""
    identifiers=await get_cam_identifiers(camera)
    camera_name=identifiers["name"]
    if camera_name and camera_name in gige_camera_nodes:
        node = gige_camera_nodes.pop(camera_name)
        node.shutdown()
        return {"status": "success", "message": f"Camera '{camera_name}' node stopped."}
    else:
        return {"status": "info", "message": "Camera node is not running."}

@router.get("/cameras")
async def get_all_cameras():
    return await get_all_gigE_cam_ids()

@router.get("/camera/{camera}")
async def get_camera(camera: int | str):

    identifiers=await get_cam_identifiers(camera)
    camera_id=identifiers["id"]
    data = await get_gigE_camera(camera_id)
    if data is None:
        raise HTTPException(status_code=404, detail=f"Camera with ID '{camera_id}' not found.")
    return data

class NotesData(BaseModel):
    notes: str


@router.post("/camera/{camera}/notes")
async def set_camera_notes(camera: int | str, data: NotesData):
    identifiers=await get_cam_identifiers(camera)
    camera_id=identifiers["id"] if identifiers["id"] is not None else camera
    if update_camera_notes(camera_id, data.notes):
                return {"status": "success", "message": "Notes updated successfully."}
    else:
        raise HTTPException(status_code=500, detail="Failed to update notes.")
    
    
    
class PresetNameData(BaseModel):
    preset_name: str

@router.post("/camera/{camera}/publishing_preset")
async def set_camera_publishing_preset(camera: int | str, data: PresetNameData):
    identifiers=await get_cam_identifiers(camera)
    camera_id=identifiers["id"] if identifiers["id"] is not None else camera
    if update_camera_publishing_preset(camera_id, data.preset_name):
        
                return {"status": "success", "message": "Publishing preset updated successfully."}
    else:
        raise HTTPException(status_code=500, detail="Failed to update publishing preset.")
    
@router.delete("/camera/{camera}")
async def delete_camera_endpoint(camera: int):
    """Deletes a camera from the system."""
    identifiers=await get_cam_identifiers(camera)
    camera_id=identifiers["id"] if identifiers["id"] is not None else camera
    camera_name = get_camera_name_by_id(camera_id)
    if camera_name and camera_name in gige_camera_nodes:
        node = gige_camera_nodes.pop(camera_name)
        node.shutdown()
    if delete_camera_from_db(camera_id):
        return {"status": "success", "message": f"Camera ID '{camera_id}' has been deleted."}
    else:
        if not get_camera_name_by_id(camera_id):
             return {"status": "success", "message": f"Camera ID '{camera_id}' already deleted."}
        raise HTTPException(status_code=500, detail="Failed to delete camera from database.")
