import gi
from gi.repository import Aravis
import subprocess
import sys
import os
import json
import ast
import psycopg2
import logging
from utils.gigE.camera_features import GigEInteractiveTool
from utils.db.db_utils import (
    get_gigE_features_from_db,
    update_gigE_feature_in_db,
    update_feature_writability,
    get_camera_name_by_id,
    get_camera_ip_by_id,
    get_db_connection,
    get_camera_id_by_name
)
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
# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
async def get_cam_identifiers(camera: int | str):
    """
    Fetches camera identifiers from the database.
    Returns a dictionary mapping camera names to their IP addresses.
    """
    try:
        identifiers={
            "id": None,
            "name": None,
            "ip_address": "localhost"
        }
        try:
            camera = int(camera)
        except ValueError:
            pass
        if isinstance(camera, int):
            identifiers["id"] = camera
            identifiers["ip"] = get_camera_ip_by_id(camera)
            identifiers["name"] = get_camera_name_by_id(camera)
        elif isinstance(camera, str):
            if camera == 'localhost':
                raise ValueError("Camera identifier cannot be 'localhost'. Please provide a valid camera name or ID.")
            
            # Attempt to get camera ID by name first
            camera_id = get_camera_id_by_name(camera)
            if camera_id != -1:
                identifiers["id"] = camera_id
                identifiers["name"] = camera
                identifiers["ip"] = get_camera_ip_by_id(camera_id)
            else:
                # If not found by name, it might be a device ID or other unique identifier.
                # This part of the logic may need to be adjusted based on how device IDs are stored.
                # For now, we assume the string is a name.
                logger.warning(f"No camera found with name '{camera}', attempting to use it as a direct identifier.")
                identifiers["name"] = camera
                # If we need to resolve an IP from a unique device ID string, we'd need a different db function.
                # For now, this path will likely fail if the name isn't in the DB.
        return identifiers
    except Exception as e:
        logger.error(f"Error fetching camera identifiers: {e}")
        return {}

async def initialize_camera_data(camera, ip):
    """
    Initializes or updates features for a given camera ID.
    This function should be called after a camera is created in the DB.
    """
    cam = GigEInteractiveTool()
    try:
        if isinstance(camera, str) and ip:
            camera_name = camera
            camera_ip = ip
        else:
            identifiers = await get_cam_identifiers(camera)
            camera_name = identifiers["name"]
            camera_ip = identifiers["ip"]

        if camera_ip is None or camera_name is None:
            logger.error("Camera IP and name could not be determined for initialization.")
            return

        logger.info(f"Initializing/updating features for {camera_name} ({camera_ip})...")

        await cam.initialize(camera_name)
        feature_details = cam.feature_groups
        
        camera_id = await create_gigE_camera(camera_name, camera_ip, feature_details)
        
        logger.info(f"Feature initialization/update for camera '{camera_name}' completed.")
        return camera_id

    except Exception as e:
        logger.error(f"Failed to initialize camera tool for feature update: {e}")
    finally:
        if hasattr(cam, 'close') and callable(getattr(cam, 'close')):
            cam.close()
        else:
            logger.warning("Camera object does not have a 'close' method.")

async def update_camera_features(camera_identifier="auto"):
    """
    Connects to the camera, fetches current feature values,
    and updates the database.
    """
    camera = None
    conn = None
    try:
        identifiers = await get_cam_identifiers(camera_identifier)
        camera_id = identifiers["id"]
        camera_name = identifiers["name"]
        
        if not camera_id:
            logger.error(f"Could not resolve camera ID for identifier: {camera_identifier}")
            return

        logger.info(f"Update Camera Features! Connecting to camera: '{camera_name}'...")
        try:
            camera = Aravis.Camera.new(camera_name)
        except Exception as e:
            logger.error(f"Error connecting to camera '{camera_name}': {e}")
            camera = None
        
        if camera is None:
            raise Exception(f"Camera '{camera_name}' not found.")
            
        logger.info(f"Connected to {camera.get_model_name()} ({camera.get_device_id()})")
        
        device = camera.props.device
        
        conn = get_db_connection()
        if not conn:
            logger.error("Cannot update features: Database connection failed.")
            return
            
        features_to_update = get_gigE_features_from_db(conn, camera_id)
        if not features_to_update:
            logger.warning("No features found in the database to update.")
            return
            
        updated_count = 0
        for feature_info in features_to_update:
            feature_name = feature_info["name"]
            feature_type = feature_info["type"]
            group_name = feature_info["group_name"]
            current_value = None
            try:
                if feature_type == "String":
                    current_value = device.get_string_feature_value(feature_name)
                elif feature_type == "Boolean":
                    current_value = device.get_boolean_feature_value(feature_name)
                elif feature_type == "Integer":
                    current_value = device.get_integer_feature_value(feature_name)
                elif feature_type == "Float":
                    current_value = device.get_float_feature_value(feature_name)
                elif feature_type == "Enumeration":
                    current_value = device.get_string_feature_value(feature_name)
                else:
                    logger.warning(f"Unsupported feature type '{feature_type}' for feature '{feature_name}'.")
                    continue
                    
                if current_value is not None:
                    update_gigE_feature_in_db(conn, camera_id, feature_name, group_name, str(current_value))
                    updated_count += 1
                    
                    # Attempt to set the feature value to determine writability
                    try:
                        if feature_type == "String":
                            device.set_string_feature_value(feature_name, current_value)
                        elif feature_type == "Boolean":
                            device.set_boolean_feature_value(feature_name, current_value == "True")
                        elif feature_type == "Integer":
                            device.set_integer_feature_value(feature_name, int(current_value))
                        elif feature_type == "Float":
                            device.set_float_feature_value(feature_name, float(current_value))
                        elif feature_type == "Enumeration":
                            device.set_string_feature_value(feature_name, current_value)
                        else:
                            logger.warning(f"Unsupported feature type '{feature_type}' for feature '{feature_name}'.")
                            continue
                        is_writable = True  # If setting the value succeeds, it's writable
                    except Exception as set_e:
                        logger.warning(f"Failed to set feature '{feature_name}': {set_e}")
                        is_writable = False # If setting the value fails, it's not writable
                    
                    update_feature_writability(conn, camera_id, feature_name, group_name, is_writable)
                else:
                    logger.warning(f"Feature '{feature_name}' (Type: {feature_type}) is not readable from the camera.")

            except Exception as e:
                logger.error(f"Error processing feature '{feature_name}' (Type: {feature_type}, Group: {group_name}): {e}")
                update_gigE_feature_in_db(conn, camera_id, feature_name, group_name, "notReadable")

        logger.info(f"Successfully updated {updated_count} features in the database.")

    except Exception as e:
        logger.error(f"Failed to update camera features: {e}")

    finally:
        if camera:
            del camera
        if conn:
            try:
                conn.close()
            except Exception as e:
                logger.error(f"Error closing database connection: {e}")
