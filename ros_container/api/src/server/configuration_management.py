import logging
from fastapi import APIRouter, HTTPException
from typing import Dict
import yaml
import os

logger = logging.getLogger(__name__)

router = APIRouter()
CONFIG_DIR = "/home/sourcingapi/config" # Directory for configuration files

@router.get("/config/{config_name}")
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

@router.post("/config/{config_name}")
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
