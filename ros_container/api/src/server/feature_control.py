import logging
from fastapi import APIRouter, HTTPException
from typing import Dict, Any
from pydantic import BaseModel
from devices.gig_e_driver import GigECameraNode
from .state import gige_camera_nodes

logger = logging.getLogger(__name__)

router = APIRouter()

class SetFeatureRequest(BaseModel):
    feature: str
    value: str

@router.post("/set_feature/{camera_id}")
async def set_feature(camera_id: str, req: SetFeatureRequest):
    node = gige_camera_nodes.get(camera_id)
    if not node:
        raise HTTPException(status_code=404, detail=f"Camera '{camera_id}' not found.")
    success, message = await node.set_feature(req.feature, req.value)
    return {"success": success, "message": message}

@router.get("/get_features/{camera_id}")
async def get_features(camera_id: str):
    node = gige_camera_nodes.get(camera_id)
    if not node:
        raise HTTPException(status_code=404, detail=f"Camera '{camera_id}' not found.")
    features = await node.get_features()
    return {"features": features}


class FeatureUpdate(BaseModel):
    feature_name: str
    value: Any
# not sure if this is still required
@router.post("/camera/{camera_id}/set_feature")
async def set_camera_feature(camera_id: str, data: Dict[str, str]):
    """
    Set a feature for a specific camera.
    Expects JSON: {"feature": "FeatureName", "value": "Value"}
    """
    node = gige_camera_nodes.get(camera_id)
    if not node or not node.is_running:
        raise HTTPException(status_code=404, detail=f"Camera node '{camera_id}' not found or not running.")
    
    feature = data.get("feature")
    value = data.get("value")
    success, message = await node.set_feature(feature, value)
    
    if not success:
         raise HTTPException(status_code=500, detail=f"Failed to set feature: {message}")

    return {"success": success, "message": message}
#
@router.get("/camera/{camera_id}/features")
async def get_camera_features(camera_id: str):
    """
    Get all features for a specific camera.
    """
    node = gige_camera_nodes.get(camera_id)
    if not node:
        raise HTTPException(status_code=404, detail=f"Camera node '{camera_id}' not found.")
    features = await node.get_features()
    return features
