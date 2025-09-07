import gi
from gi.repository import Aravis
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

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

async def initialize_camera_data(camera_id: int):
    """
    Initializes or updates features for a given camera ID.
    This function should be called after a camera is created in the DB.
    """
    camera_name = get_camera_name_by_id(camera_id)
    camera_ip = get_camera_ip_by_id(camera_id)

    if not camera_name:
        logger.error(f"No camera found with ID {camera_id} to initialize.")
        return

    logger.info(f"Initializing/updating features for {camera_name} ({camera_ip})...")

    cam = GigEInteractiveTool()
    try:
        identifier = camera_ip if camera_ip and camera_ip != 'localhost' else camera_name
        await cam.initialize(identifier)

        # Here, we would ideally pass the feature details to a function that populates the features table.
        # The current `create_gigE_camera` also handles this, which is a bit of a code smell.
        # For now, we will rely on a separate update mechanism.
        update_camera_features(identifier)

        logger.info(f"Feature initialization/update for camera ID {camera_id} completed.")
    except Exception as e:
        logger.error(f"Failed to initialize camera tool for feature update: {e}")
    finally:
        cam.close()

def update_camera_features(camera_identifier="auto"):
    """
    Connects to the camera, fetches current feature values,
    and updates the database.
    """
    camera = None
    conn = get_db_connection()
    if not conn:
        logger.error("Cannot update features: Database connection failed.")
        return

    try:
        logger.info(f"Update Camera Features! Connecting to camera: '{camera_identifier}'...")
        camera = Aravis.Camera.new(camera_identifier)
        if camera is None:
            raise Exception(f"Camera '{camera_identifier}' not found.")

        logger.info(f"Connected to {camera.get_model_name()} ({camera.get_device_id()})")
        device = camera.props.device

        features_to_update = get_gigE_features_from_db(conn)
        if not features_to_update:
            logger.warning("No features found in the database to update.")
            return

        updated_count = 0
        for feature_info in features_to_update:
            feature_name = feature_info["name"]
            feature_type = feature_info["type"]
            group_name = feature_info["group_name"]
            try:
                current_value = None
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

                if current_value is not None:
                    update_gigE_feature_in_db(conn, feature_name, group_name, str(current_value))
                    updated_count += 1
                    is_writable = device.is_feature_writable(feature_name)
                    update_feature_writability(conn, feature_name, group_name, is_writable)
            except Exception as e:
                logger.error(f"Error processing feature '{feature_name}': {e}")
                update_gigE_feature_in_db(conn, feature_name, group_name, "notReadable")

        logger.info(f"Successfully updated {updated_count} features in the database.")

    except Exception as e:
        logger.error(f"Failed to update camera features: {e}")
    finally:
        if camera: del camera
        if conn: conn.close()
