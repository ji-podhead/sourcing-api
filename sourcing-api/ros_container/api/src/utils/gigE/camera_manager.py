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
from utils.db.db_utils import get_all_gigE_cam_ids, get_gigE_camera, get_db_connection, gigE_camera_exists, create_gigE_camera, get_gigE_features_from_db, update_gigE_feature_in_db, update_feature_writability
# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


async def initialize_camera_data(camera_id=None, camera_protocol=None):
    """
    Checks if camera features are initialized in the database.
    If not, it attempts to create entries for new cameras and then
    populates the database with their features.
    If already initialized, it runs the update_camera_features.py script
    to update the feature values in the database.
    """
    conn = get_db_connection()
    if not conn:
        logger.error("Skipping camera data handling due to database connection failure.")
        return

    try:
        with conn.cursor() as cur:
            # Check if camera_initialized table exists
            # cur.execute("SELECT EXISTS (SELECT 1 FROM information_schema.tables WHERE table_schema = 'public' AND table_name = 'camera_initialized');")
            # table_exists = cur.fetchone()[0]
            cam_exists= await gigE_camera_exists(camera_id) if camera_id else False
            if cam_exists:
                update_camera_features(camera_id)
            else:
                logger.info("Camera features not initialized. Processing cameras...")
                cam= GigEInteractiveTool()
                await cam.initialize(camera_id)
                await create_gigE_camera(camera_id,cam)
                logger.info("successfully created cam, updating values")

    except Exception as e:
        logger.error(f"An unexpected error occurred during camera data handling: {e}")
        if conn:
            conn.rollback()
        # Ensure it's marked as not initialized if a general error occurs
        try:
            # Check if table exists before attempting insert/update
            cur.execute("SELECT EXISTS (SELECT 1 FROM information_schema.tables WHERE table_schema = 'public' AND table_name = 'camera_initialized');")
            if cur.fetchone()[0]:
                cur.execute("INSERT INTO camera_initialized (initialized) VALUES (FALSE) ON CONFLICT (initialized) DO UPDATE SET initialized = EXCLUDED.initialized;")
                conn.commit()
                logger.info("Marking as not initialized due to unexpected error.")
            else:
                logger.warning("Camera initialization table 'camera_initialized' not found. Cannot mark as not initialized.")
        except Exception as commit_e:
            logger.error(f"Error marking camera_initialized as FALSE during general error handling: {commit_e}")
            conn.rollback()
    finally:
        if conn:
            conn.close()



def update_camera_features(camera_identifier="auto"):
    """
    Connects to the camera, fetches current feature values,
    and updates the database.
    """
    camera = None
    try:
        logger.info(f"Update Camera Features! Connecting to camera: '{camera_identifier}'...")
        camera = Aravis.Camera.new(camera_identifier)
        if camera is None:
            raise Exception(f"Camera '{camera_identifier}' not found.")
        logger.info(f"Connected to {camera.get_model_name()} ({camera.get_device_id()})")
        device = camera.props.device # Get the device object to access feature methods
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
                    is_writable = False
                    try:
                        if feature_type == "String":
                            device.set_string_feature_value(feature_name, current_value)
                            is_writable = True
                        elif feature_type == "Boolean":
                            device.set_boolean_feature_value(feature_name, current_value)
                            is_writable = True
                        elif feature_type == "Integer":
                            device.set_integer_feature_value(feature_name, current_value)
                            is_writable = True
                        elif feature_type == "Float":
                            device.set_float_feature_value(feature_name, current_value)
                            is_writable = True
                        elif feature_type == "Enumeration":
                            device.set_string_feature_value(feature_name, current_value)
                            is_writable = True
                    except Exception:
                        is_writable = False
                    
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
            conn.close()
