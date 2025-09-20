import logging
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import json
import os

logger = logging.getLogger(__name__)
router = APIRouter()

CONFIG_FILE_PATH = "/home/sourcingapi/config/dashboard_settings.json"

class IntervalSettings(BaseModel):
    discovery_interval: int
    refresh_interval: int

def _load_settings():
    if os.path.exists(CONFIG_FILE_PATH):
        with open(CONFIG_FILE_PATH, 'r') as f:
            return json.load(f)
    return {"discovery_interval": 10000, "refresh_interval": 5000} # Default values

def _save_settings(settings):
    os.makedirs(os.path.dirname(CONFIG_FILE_PATH), exist_ok=True)
    with open(CONFIG_FILE_PATH, 'w') as f:
        json.dump(settings, f, indent=4)

@router.get("/settings/intervals", response_model=IntervalSettings)
async def get_intervals():
    """Gets the discovery and refresh intervals."""
    settings = _load_settings()
    return IntervalSettings(
        discovery_interval=settings.get("discovery_interval", 10000),
        refresh_interval=settings.get("refresh_interval", 5000)
    )

@router.post("/settings/intervals")
async def set_intervals(data: IntervalSettings):
    """Sets the discovery and refresh intervals."""
    if data.discovery_interval < 1000 or data.refresh_interval < 1000:
        raise HTTPException(status_code=400, detail="Intervals must be at least 1000 ms.")
    settings = _load_settings()
    settings["discovery_interval"] = data.discovery_interval
    settings["refresh_interval"] = data.refresh_interval
    _save_settings(settings)
    return {"status": "success", "message": "Intervals updated successfully."}
