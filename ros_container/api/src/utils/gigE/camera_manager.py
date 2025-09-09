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
    get_db_connection
)
# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
async def initialize_camera_data(camera_id: int):
    """
    Initializes or updates features for a given camera ID.
    This function should be called after a camera is created in the DB.
    """
    camera_name = get_camera_name_by_id(camera_id)
    camera_ip = get_camera_ip_by_id(camera_id)
    print(f"initialize_camera_data called for camera ID {camera_id} with name '{camera_name}' and IP '{camera_ip}'")
    if not camera_name:
        logger.error(f"No camera found with ID {camera_id} to initialize.")
        return

    logger.info(f"Initializing/updating features for {camera_name} ({camera_ip})...")

    cam = GigEInteractiveTool()
    retries = 3
    for attempt in range(retries):
        try:
            identifier = camera_ip if camera_ip and camera_ip != 'localhost' else camera_name
            await cam.initialize(identifier)
            
            # Here, we would ideally pass the feature details to a function that populates the features table.
            # The current `create_gigE_camera` also handles this, which is a bit of a code smell.
            # For now, we will rely on a separate update mechanism.
            update_camera_features(identifier)

            logger.info(f"Feature initialization/update for camera ID {camera_id} completed.")
            break # If successful, break out of the retry loop
        except Exception as e:
            logger.error(f"Failed to initialize camera tool for feature update (Attempt {attempt + 1}/{retries}): {e}")
            if attempt == retries - 1:
                logger.error(f"Failed to initialize camera tool for feature update after {retries} attempts.")
        finally:
            cam.close()

def update_camera_features(camera_identifier="auto"):
    """
    Connects to the camera, fetches current feature values,
    and updates the database.
    """
    camera = None
    try:
        logger.info(f"Update Camera Features! Connecting to camera: '{camera_identifier}'...")
        try:
            camera = Aravis.Camera.new(camera_identifier)
        except Exception as e:
            logger.error(f"Error connecting to camera '{camera_identifier}': {e}")
            camera = None
        if camera is None:
            raise Exception(f"Camera '{camera_identifier}' not found.")
        logger.info(f"Connected to {camera.get_model_name()} ({camera.get_device_id()})")
        try:
            device = camera.props.device # Get the device object to access feature methods
        except Exception as e:
            logger.error(f"Error getting device properties: {e}")
            raise e
        conn = get_db_connection()
        if not conn:
            logger.error("Cannot update features: Database connection failed.")
            return
        features_to_update = get_gigE_features_from_db(conn)
        #logger.info(f"fetched features from db: {features_to_update}")
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
                    # For enumerations, get_string_feature_value is typically used
                    current_value = device.get_string_feature_value(feature_name)
                else:
                    logger.warning(f"Unsupported feature type '{feature_type}' for feature '{feature_name}'.")
                    continue
                if current_value is not None:
                    update_gigE_feature_in_db(conn, feature_name, group_name, str(current_value))
                    updated_count += 1

                    # Test for writability
                    is_writable = device.is_feature_writable(feature_name)
                    update_feature_writability(conn, feature_name, group_name, is_writable)

                else:
                    logger.warning(f"Feature '{feature_name}' (Type: {feature_type}) is not readable from the camera.")

            except Exception as e:
                logger.error(f"Error processing feature '{feature_name}' (Type: {feature_type}, Group: {group_name}): {e}")
                # Optionally, try to update the DB with an error indicator or 'notReadable'
                update_gigE_feature_in_db(conn, feature_name, group_name, "notReadable")

        logger.info(f"Successfully updated {updated_count} features in the database.")

    except Exception as e:
        logger.error(f"Failed to update camera features: {e}")

    finally:
        if camera:
            del camera # Release camera resources
        if conn:
            try:
                conn.close()
            except Exception as e:
                logger.error(f"Error closing database connection: {e}")
