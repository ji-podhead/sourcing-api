import logging
from fastapi import APIRouter, HTTPException
from typing import Dict
from gi.repository import Aravis
from utils.gigE.camera_manager import initialize_camera_data
from utils.db.db_utils import (
    get_all_gigE_cams,
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

logger = logging.getLogger(__name__)
router = APIRouter()

@router.get("/discover_cameras")
async def discover_cameras():
    """Discovers all available GigE cameras on the network."""
    try:
        Aravis.update_device_list()
        n_devices = Aravis.get_n_devices()
        if n_devices == 0:
            return []

        discovered_cameras = []
        for i in range(n_devices):
            try:
                device_id = Aravis.get_device_id(i)
                cam = Aravis.Camera.new(device_id)
                if not cam:
                    continue

                name = cam.get_model_name()
                ip_address = "N/A"
                try:
                    ip_address = cam.get_string_feature_value("GevDeviceIPAddress")
                except Exception:
                    pass

                existing_id = await find_camera_by_identifier(name)
                if not existing_id and ip_address != "N/A":
                    existing_id = await find_camera_by_identifier(ip_address)

                discovered_cameras.append({
                    "name": name,
                    "ip_address": ip_address,
                    "device_id": device_id,
                    "is_in_db": existing_id is not None,
                    "db_id": existing_id
                })
            except Exception as e:
                logger.error(f"Error processing device index {i}: {e}")
                continue
        return discovered_cameras
    except Exception as e:
        logger.error(f"Error during camera discovery: {e}")
        raise HTTPException(status_code=500, detail="Failed to discover cameras.")

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

        camera_name = cam.get_model_name()
        camera_ip = "localhost"
        try:
            camera_ip = cam.get_string_feature_value("GevDeviceIPAddress")
        except Exception:
            pass

        existing_id = await find_camera_by_identifier(camera_name)
        if existing_id:
            return await get_gigE_camera(existing_id)

        new_camera_id = await create_gigE_camera(camera_name, camera_ip, {})
        if new_camera_id is None:
            raise HTTPException(status_code=500, detail="Failed to create camera in database.")
            
        await initialize_camera_data(camera_id=new_camera_id)

        return await get_gigE_camera(new_camera_id)
    except Exception as e:
        logger.error(f"Error creating camera with identifier '{identifier}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/camera/{camera_id}/start")
async def start_camera(camera_id: int):
    """Starts the camera node for publishing."""
    camera_name = get_camera_name_by_id(camera_id)
    if not camera_name:
        raise HTTPException(status_code=404, detail=f"Camera with ID {camera_id} not found.")

    if camera_name in gige_camera_nodes:
        return {"status": "info", "message": "Camera node is already running."}

    node = GigECameraNode(camera_id=camera_name, ros_host='localhost')
    node.start()
    gige_camera_nodes[camera_name] = node

    return {"status": "success", "message": f"Camera '{camera_name}' node started successfully."}

@router.post("/camera/{camera_id}/stop")
async def stop_camera(camera_id: int):
    """Stops the camera node."""
    camera_name = get_camera_name_by_id(camera_id)
    if camera_name and camera_name in gige_camera_nodes:
        node = gige_camera_nodes.pop(camera_name)
        node.shutdown()
        return {"status": "success", "message": f"Camera '{camera_name}' node stopped."}
    else:
        return {"status": "info", "message": "Camera node is not running."}

@router.get("/cameras")
async def get_all_cameras():
    return await get_all_gigE_cams()

@router.get("/camera/{camera_id}")
async def get_camera(camera_id: int):
    data = await get_gigE_camera(camera_id)
    if data is None:
        raise HTTPException(status_code=404, detail=f"Camera with ID '{camera_id}' not found.")
    return data

class NotesData(BaseModel):
    notes: str

@router.post("/camera/{camera_id}/notes")
async def set_camera_notes(camera_id: int, data: NotesData):
    if update_camera_notes(camera_id, data.notes):
        return {"status": "success", "message": "Notes updated successfully."}
    else:
        raise HTTPException(status_code=500, detail="Failed to update notes.")

class PresetNameData(BaseModel):
    preset_name: str

@router.post("/camera/{camera_id}/publishing_preset")
async def set_camera_publishing_preset(camera_id: int, data: PresetNameData):
    if update_camera_publishing_preset(camera_id, data.preset_name):
        return {"status": "success", "message": "Publishing preset updated successfully."}
    else:
        raise HTTPException(status_code=500, detail="Failed to update publishing preset.")

@router.delete("/camera/{camera_id}")
async def delete_camera_endpoint(camera_id: int):
    """Deletes a camera from the system."""
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
