import logging
from fastapi import APIRouter, HTTPException
from typing import List, Dict, Optional, Any
from pydantic import BaseModel
import yaml
import os
from .state import gige_camera_nodes
from utils.db.db_utils import (
    create_preset,
    get_preset,
    get_presets_for_device,
    update_preset,
    delete_preset,
)

logger = logging.getLogger(__name__)

router = APIRouter()
CONFIG_DIR = "/home/sourcingapi/config" # Directory for configuration files

class PresetData(BaseModel):
    name: str

@router.post("/camera/{camera_id}/save_preset")
async def save_camera_preset(camera_id: str, preset_data: PresetData):
    if camera_id not in gige_camera_nodes:
        raise HTTPException(status_code=404, detail=f"Camera '{camera_id}' not running.")
    
    node = gige_camera_nodes.get(camera_id)
    features = await node.get_features()
    
    if not features:
        raise HTTPException(status_code=500, detail="Could not retrieve camera features to save preset.")

    # Using the existing utility to create a preset
    if create_preset(camera_id, preset_data.name, features):
        return {"status": "success", "message": f"Preset '{preset_data.name}' created for camera '{camera_id}'."}
    else:
        raise HTTPException(status_code=500, detail=f"Failed to create preset '{preset_data.name}'.")
    
# --- Preset Management Endpoints ---
@router.post("/presets/{device_identifier}")
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

@router.get("/presets/{device_identifier}")
async def get_presets_for_device_endpoint(device_identifier: str):
    """Retrieves all presets for a given device."""
    presets = await get_presets_for_device(device_identifier)
    if presets is not None:
        return {"device_identifier": device_identifier, "presets": presets}
    else:
        # This case should ideally not happen if get_presets_for_device handles errors internally
        raise HTTPException(status_code=500, detail=f"Failed to retrieve presets for device '{device_identifier}'.")

@router.get("/presets/{device_identifier}/{preset_name}")
async def get_preset_endpoint(device_identifier: str, preset_name: str):
    """Retrieves a specific preset by device identifier and preset name."""
    preset_config = await get_preset(device_identifier, preset_name)
    if preset_config:
        return {"device_identifier": device_identifier, "preset_name": preset_name, "configuration": preset_config}
    else:
        raise HTTPException(status_code=404, detail=f"Preset '{preset_name}' not found for device '{device_identifier}'.")

@router.put("/presets/{device_identifier}/{preset_name}")
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

@router.delete("/presets/{device_identifier}/{preset_name}")
async def delete_preset_endpoint(device_identifier: str, preset_name: str):
    """Deletes a preset."""
    if delete_preset(device_identifier, preset_name):
        return {"status": "success", "message": f"Preset '{preset_name}' deleted for device '{device_identifier}'."}
    else:
        raise HTTPException(status_code=404, detail=f"Preset '{preset_name}' not found for device '{device_identifier}'.")

@router.post("/presets/load_from_yaml")
async def load_presets_from_yaml():
    """Loads presets from the default 'presets.yaml' file in the config directory."""
    filename = "presets.yaml"
    file_path = os.path.join(CONFIG_DIR, filename)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail=f"Default preset file '{filename}' not found in config directory.")
    
    try:
        with open(file_path, 'r') as f:
            presets_data = yaml.safe_load(f)
        
        if not isinstance(presets_data, list):
            raise HTTPException(status_code=400, detail="YAML file should contain a list of presets.")

        created_count = 0
        for preset in presets_data:
            device_identifier = preset.get("device_identifier")
            preset_name = preset.get("name")
            configuration = preset.get("configuration")

            if not all([device_identifier, preset_name, configuration]):
                logger.warning(f"Skipping preset due to missing data: {preset}")
                continue

            if create_preset(device_identifier, preset_name, configuration):
                created_count += 1
        
        return {"status": "success", "message": f"Successfully loaded {created_count} presets from '{filename}'."}

    except yaml.YAMLError as e:
        raise HTTPException(status_code=500, detail=f"Error parsing YAML file: {e}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to load presets from YAML: {e}")

@router.post("/presets/{device_identifier}/{preset_name}/apply")
async def apply_preset_endpoint(device_identifier: str, preset_name: str):
    """Applies a preset to a given device."""
    preset_config = await get_preset(device_identifier, preset_name)
    if not preset_config:
        raise HTTPException(status_code=404, detail=f"Preset '{preset_name}' not found for device '{device_identifier}'.")

    camera_node = get_gigE_camera(device_identifier)
    if not camera_node:
        raise HTTPException(status_code=404, detail=f"Camera '{device_identifier}' not found or not running.")

    for group_name, features in preset_config.items():
        for feature_name, feature_data in features.items():
            value = feature_data.get("value")
            await camera_node.set_feature(feature_name, value)

    return {"status": "success", "message": f"Preset '{preset_name}' applied to device '{device_identifier}'."}
