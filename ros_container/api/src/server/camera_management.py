import gi
gi.require_version('Aravis', '0.8')
import logging
from typing import Dict
from gi.repository import Aravis
from utils.gigE.camera_manager import initialize_camera_data
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

logger = logging.getLogger(__name__)

router = APIRouter()

CONFIG_DIR = "/home/sourcingapi/config" # Directory for configuration files


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
                device = cam.props.device
                gv_device = device.get_device()
                try:
                    ip_address = gv_device.get_current_ip().ip.to_string()
            
                    print(f"Camera IP: {ip_address}")
                except Exception as e:
                    logger.error(f"Error getting IP address for device {device_id}: {e}")
                    ip_address = "N/A"

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

            camera_name = cam.get_model_name()
            camera_ip = ""
            try:
                device = cam.props.device
                for key,val in cam.props.items():
                        print(f"Property: {key} = {val}")
                camera_ip = device.get_string_feature_value("GevDeviceIPAddress")
            except Exception as e:
                logger.error(f"Error getting IP address for device {identifier}: {e}")
                camera_ip = ""
            print(f"Camera IP: {camera_ip}")

            existing_id = await find_camera_by_identifier(camera_name)
            if existing_id:
                return await get_gigE_camera(existing_id)

            new_camera_id = await create_gigE_camera(camera_name, camera_ip, {})
            if new_camera_id is None:
                raise HTTPException(status_code=500, detail="Failed to create camera in database.")

            logger.info(f"Calling initialize_camera_data for camera ID {new_camera_id}")
            await initialize_camera_data(new_camera_id)
            logger.info(f"initialize_camera_data completed for camera ID {new_camera_id}")
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
    return await get_all_gigE_cam_ids()

@router.get("/camera/{camera_id}")
async def get_camera(camera_id: int):
    data = await get_gigE_camera(camera_id)
    if data is None:
        raise HTTPException(status_code=404, detail=f"Camera with ID '{camera_id}' not found.")
    return data

# @router.get("/get_cam/{protocoll}/{camera_identifier}")
# async def get_cam(protocoll: str, camera_identifier:str):
#     if(protocoll == "gigE"):
#         data= await get_gigE_camera(camera_identifier)
#         if(data is None):
#             raise HTTPException(status_code=404, detail=f"Camera '{camera_identifier}' not found.")
#         else:
#             return data
#     else:
#         raise HTTPException(status_code=404, detail=f"Protocol '{protocoll}' not supported.")

# @router.get("/get_all_cams/{protocol}")
# async def get_all_cams(protocol: str):
#     # Strip any leading/trailing single quotes from the protocol string
#     protocol = protocol.strip("'")
#     if(protocol == "gigE"):
#         stored_cameras = await get_all_gigE_cam_ids()
#         logger.info(f"Stored cameras found in DB: {[cam for cam in stored_cameras]}")
#         all_camera_data = []
#         for cam_id in stored_cameras:
#             logger.info(f"Updating camera data for gigE camera with ID: {cam_id}")
#             await update_cam("gigE", cam_id) # Ensure features are up-to-date
            
#             camera_details = await get_gigE_camera(cam_id)
#             if camera_details:
#                 # Augment camera_details with frontend-expected fields
#                 augmented_camera_data = {
#                     **camera_details,
#                     "name": f"Camera: {camera_details['identifier']}",
#                     "path": "/imagingsource",  # Generic path for dynamic cameras
#                     "apiEndpoint": "/driver/camera",  # Generic API endpoint for dynamic cameras
#                     "fileName": f"camera_{camera_details['id']}.yaml",  # Placeholder for config file name
#                 }
#                 all_camera_data.append(augmented_camera_data)
#         return all_camera_data
#     else:
#         raise HTTPException(status_code=404, detail=f"Protocol '{protocol}' not supported.")
#         return None

# @router.post("/update_cam/{protocoll}/{camera_identifier}")
# async def update_cam(protocoll: str, camera_identifier:str):
#     if(protocoll == "gigE"):
#         await initialize_camera_data(camera_identifier)

# @router.post("/create_camera")
# async def create_camera(camera_data: Dict[str, str]):
#     """
#     Creates a new camera based on provided ID and protocol.
#     """
#     logger.info(f"Received camera data for creation: {camera_data}") # Log received data
#     protocol = camera_data.get("protocol")
#     camera_id = camera_data.get("id") # Frontend sends 'id'

#     if not protocol or not camera_id:
#         raise HTTPException(status_code=400, detail="Protocol and camera ID are required.")

#     logger.info(f"Received request to create camera: Protocol={protocol}, ID={camera_id}")
#     try:
#         if protocol == "gigE":
#             # if not is_valid_ip(camera_id):
#             #     raise HTTPException(status_code=400, detail=f"Invalid IP address format for gigE camera: {camera_id}")
            
#             existing_camera = await get_gigE_camera(camera_id)
#             if not existing_camera:
#                 await initialize_camera_data(camera_id)
            
#             if camera_id in gige_camera_nodes:
#                 return {"status": "info", "message": f"Camera '{camera_id}' is already running."}

#             config_file_name = f"gige_{camera_id.replace('.', '_')}.yml"
#             config_file_path = os.path.join(CONFIG_DIR, config_file_name)
            
#             if not os.path.exists(config_file_path):
#                 default_config = {
#                     "guid": None,
#                     "ImageFormatControl": {"PixelFormat": "Mono8"},
#                     "AcquisitionControl": {"AcquisitionMode": "Continuous"}
#                 }
#                 with open(config_file_path, 'w') as f:
#                     yaml.dump(default_config, f)
            
#             # Erstelle eine Instanz unserer NEUEN Klasse
#             # Annahme: roslibpy läuft auf demselben Host (im selben Docker-Netzwerk)
#             node = GigECameraNode(camera_id=camera_id, ros_host='localhost') # oder IP des ROS-Containers
#             node.start() # Startet den Hintergrund-Thread für die ROS-Kommunikation
            
#             gige_camera_nodes[camera_id] = node
            
#             return {"status": "success", "message": f"GigE camera '{camera_id}' created and started successfully."}


#         elif protocol == "GMSL2":
#             # Placeholder for GMSL2 camera handling
#             # This would require specific logic for GMSL2 cameras, potentially using different drivers or initialization functions.
#             # For now, we'll return an error indicating it's not yet supported.
#             logger.warning(f"GMSL2 protocol is not yet supported for camera creation.")
#             raise HTTPException(status_code=501, detail=f"GMSL2 protocol is not yet supported.")
#         else:
#             raise HTTPException(status_code=400, detail=f"Protocol '{protocol}' not supported for creating cameras.")
#     except HTTPException as e:
#         raise e
#     except Exception as e:
#         logger.error(f"Error creating/updating camera '{camera_id}' with protocol '{protocol}': {e}")
#         raise HTTPException(status_code=500, detail=f"Error creating/updating camera: {str(e)}")

class NotesData(BaseModel):
    notes: str

# class PresetNameData(BaseModel):
#     preset_name: str

# @router.post("/camera/{camera_identifier}/notes")
# async def set_camera_notes(camera_identifier: str, data: NotesData):
#     if update_camera_notes(camera_identifier, data.notes):
#         return {"status": "success", "message": "Notes updated successfully."}
#     else:
#         raise HTTPException(status_code=500, detail="Failed to update notes.")

# @router.post("/camera/{camera_identifier}/publishing_preset")
# async def set_camera_publishing_preset(camera_identifier: str, data: PresetNameData):
#     if update_camera_publishing_preset(camera_identifier, data.preset_name):
#         return {"status": "success", "message": "Publishing preset updated successfully."}
#     else:
#         raise HTTPException(status_code=500, detail="Failed to update publishing preset.")

# @router.post("/delete_camera")
# async def delete_camera(camera_data: Dict[str, str]):
#     camera_id = camera_data.get("id")
#     if not camera_id:
#         raise HTTPException(status_code=400, detail="camera_id is required.")

#     if camera_id in gige_camera_nodes:
#         node = gige_camera_nodes.pop(camera_id)
#         node.shutdown() # Wichtig: Sauberes Herunterfahren!

#     return {"status": "success", "message": f"Camera '{camera_id}' has been stopped."}

# # Helper function for IP validation (can be expanded for more robust checks)
# def is_valid_ip(ip_address: str) -> bool:
#     import ipaddress
#     try:
#         ipaddress.ip_address(ip_address)
#         return True
#     except ValueError:
#         return False


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
