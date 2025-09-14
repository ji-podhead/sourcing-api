from fastapi import APIRouter
from server import camera_management, configuration_management, driver_management, feature_control, logs, preset_management, recording_management, terminal

router = APIRouter()

router.include_router(camera_management.router, tags=["camera_management"])
router.include_router(configuration_management.router, tags=["configuration_management"])
router.include_router(driver_management.router, tags=["driver_management"])
router.include_router(feature_control.router, tags=["feature_control"])
router.include_router(logs.router, tags=["logs"])
router.include_router(preset_management.router, tags=["preset_management"])
router.include_router(recording_management.router, tags=["recording_management"])
router.include_router(terminal.router, tags=["terminal"])
