#!/usr/bin/env python3
import gi
import sys
import os
import json
import psycopg2
import logging
from utils.gigE.camera_features import XenicsInteractiveTool
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


# Database configuration (should match main.py)
DB_HOST = "localhost"
DB_PORT = "5432"
DB_NAME = "camera_db"
DB_USER = "user"
DB_PASSWORD = "password"

def get_db_connection():
    """Establishes a connection to the PostgreSQL database."""
    try:
        conn = psycopg2.connect(
            host=DB_HOST,
            port=DB_PORT,
            dbname=DB_NAME,
            user=DB_USER,
            password=DB_PASSWORD
        )
        return conn
    except psycopg2.OperationalError as e:
        logger.error(f"Database connection failed: {e}")
        return None

async def gigE_camera_exists(camera_identifier: str) -> bool:
    """Checks if a camera with the given identifier exists in the database."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot check camera existence.")
        return False

    try:
        with conn.cursor() as cur:
            cur.execute("SELECT COUNT(*) FROM cameras WHERE identifier = %s;", (camera_identifier,))
            count = cur.fetchone()[0]
            return count > 0
    except Exception as e:
        logger.error(f"Error checking camera existence: {e}")
        return False
    finally:
        if conn:
            conn.close()


def get_gigE_features_from_db(conn):
    """Fetches feature names and types from the database."""
    features_to_update = []
    try:
        with conn.cursor() as cur:
            cur.execute("""
                SELECT f.name, f.type, fg.name as group_name
                FROM features f
                JOIN feature_groups fg ON f.group_id = fg.id;
            """)
            for row in cur.fetchall():
                feature_name, feature_type, group_name = row
                features_to_update.append({
                    "name": feature_name,
                    "type": feature_type,
                    "group_name": group_name
                })
        logger.info(f"Fetched {len(features_to_update)} features from database.")
        return features_to_update
    except Exception as e:
        logger.error(f"Error fetching features from database: {e}")
        return []

def update_gigE_feature_in_db(conn, feature_name, group_name, new_value):
    """Updates a specific feature's value in the database."""
    try:
        with conn.cursor() as cur:
            cur.execute("""
                UPDATE features
                SET value = %s
                WHERE name = %s AND group_id = (SELECT id FROM feature_groups WHERE name = %s);
            """, (new_value, feature_name, group_name))
            conn.commit()
            # logger.debug(f"Updated feature '{feature_name}' in group '{group_name}' with value '{new_value}'.")
    except Exception as e:
        logger.error(f"Error updating feature '{feature_name}' in group '{group_name}': {e}")
        conn.rollback()


async def create_gigE_camera(camera_identifier: str, details: dict):
    """Creates a new camera entry and populates its features in the database."""
    conn = get_db_connection()
    camera_protocol = "gigE"
    camera_id = camera_identifier
    if not conn:
        logger.error("Database connection failed. Cannot create camera.")
        return False

    try:
        with conn.cursor() as cur:
            cur.execute("SELECT identifier FROM cameras;")
            camera_rows = cur.fetchall()
            if not camera_rows and camera_id:
                logger.info(f"Attempting to create entry for camera ID: {camera_id} with protocol: {camera_protocol}")
                try:
                    default_config = json.dumps({"protocol": camera_protocol, "ip": camera_id})
                    cur.execute(
                        "INSERT INTO cameras (identifier, type, config) VALUES (%s, %s, %s) ON CONFLICT (identifier) DO NOTHING;",
                        (camera_id, camera_protocol, default_config)
                    )
                    conn.commit()
                    logger.info(f"Successfully created entry for camera ID: {camera_id}")
                    cur.execute("SELECT identifier FROM cameras;")
                    camera_rows = cur.fetchall()
                except Exception as insert_e:
                    logger.error(f"Error creating camera entry for {camera_id}: {insert_e}")
                    conn.rollback()
                    return
            elif not camera_rows and not camera_id:
                logger.warning("No camera identifiers found in the database and no specific camera data provided for initialization.")
                return

            if not camera_rows:
                logger.error("No cameras available for initialization after processing.")
                return

            for row in camera_rows:
                current_camera_identifier = row[0]

                if camera_id is not None and current_camera_identifier != camera_id:
                    continue

                logger.info(f"Initializing features for camera: {current_camera_identifier}...")
                feature_data = details.feature_groups
                logger.debug(f"Feature data for {current_camera_identifier} extracted.")

                for group_name, group_details in feature_data.items():
                    logger.info(f"Processing feature group: {group_name}")
                    cur.execute("INSERT INTO feature_groups (name) VALUES (%s) ON CONFLICT (name) DO NOTHING RETURNING id;", (group_name,))
                    group_id_row = cur.fetchone()
                    if group_id_row:
                        group_id = group_id_row[0]
                    else:
                        cur.execute("SELECT id FROM feature_groups WHERE name = %s;", (group_name,))
                        group_id_row = cur.fetchone()
                        if not group_id_row:
                            logger.warning(f"Could not get or create group ID for '{group_name}'. Skipping features in this group.")
                            continue
                        group_id = group_id_row[0]

                    # --- FIX START ---
                    # The `group_details` dictionary contains a key "features" which holds the actual dictionary of features.
                    actual_features_dict = group_details.get("features")
                    if not actual_features_dict or not isinstance(actual_features_dict, dict):
                        logger.warning(f"Group '{group_name}' does not contain a valid 'features' dictionary. Skipping.")
                        continue

                    # Now, correctly iterate through the actual features
                    for feature_name, feature_details in actual_features_dict.items():
                        try:
                            feature_type = feature_details.get("type")
                            value = feature_details.get("value")
                            tooltip = feature_details.get("tooltip", "")
                            description = feature_details.get("description", "")

                            # Ensure the value is a string representation for DB storage
                            # Using json.dumps is safe for complex types like lists or dicts
                            if isinstance(value, (dict, list)):
                                value_str = json.dumps(value)
                            else:
                                value_str = str(value)

                            # Insert or update the feature in the database
                            cur.execute("""
                                INSERT INTO features (name, type, value, tooltip, description, group_id)
                                VALUES (%s, %s, %s, %s, %s, %s)
                                ON CONFLICT (name, group_id) DO UPDATE SET
                                    type = EXCLUDED.type,
                                    value = EXCLUDED.value,
                                    tooltip = EXCLUDED.tooltip,
                                    description = EXCLUDED.description;
                            """, (feature_name, feature_type, value_str, tooltip, description, group_id))
                        except Exception as feature_e:
                            logger.error(f"Error processing feature '{feature_name}' in group '{group_name}': {feature_e}")
                            conn.rollback() # Rollback this specific feature's transaction
                        else:
                            conn.commit() # Commit after each successful feature insertion
                    # --- FIX END ---
        logger.info(f"Feature initialization for camera '{camera_id}' completed.")
    except Exception as e:
        logger.error(f"An unexpected error occurred during camera creation/initialization: {e}")
        if conn:
            conn.rollback()
    finally:
        if conn:
            conn.close()
            
async def get_all_gigE_cam_ids():
    """Retrieves all camera identifiers from the database."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot retrieve camera IDs.")
        return []

    camera_ids = []
    try:
        with conn.cursor() as cur:
            cur.execute("SELECT identifier FROM cameras;")
            rows = cur.fetchall()
            for row in rows:
                camera_ids.append(row[0])
        logger.info(f"Retrieved {len(camera_ids)} camera IDs from the database.")
    except Exception as e:
        logger.error(f"Error retrieving camera IDs: {e}")
    finally:
        if conn:
            conn.close()
    return camera_ids

async def get_gigE_camera(camera_identifier: str):
    """Fetches camera details and its features from the database."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot retrieve camera details.")
        return None

    try:
        with conn.cursor() as cur:
            cur.execute("SELECT id, identifier, type, config FROM cameras WHERE identifier = %s;", (camera_identifier,))
            camera_row = cur.fetchone()

            if not camera_row:
                logger.warning(f"No camera found with identifier: {camera_identifier}")
                return None

            camera_data = {
                "id": camera_row[0],
                "identifier": camera_row[1],
                "type": camera_row[2],
                "config": camera_row[3],
                "status": "stopped",  # Default status, frontend will update dynamically
                "features": []
            }

            # Fetch feature groups and their features for this camera
            cur.execute("""
                SELECT
                    fg.name AS group_name,
                    f.name AS feature_name,
                    f.description,
                    f.type,
                    f.value,
                    f.min,
                    f.max,
                    f.options,
                    f.representation
                FROM
                    feature_groups fg
                JOIN
                    features f ON fg.id = f.group_id
                ORDER BY
                    fg.name, f.name;
            """)
            feature_rows = cur.fetchall()

            feature_groups_dict = {}
            for row in feature_rows:
                group_name, feature_name, description, f_type, value, min_val, max_val, options_str, representation = row
                
                if group_name not in feature_groups_dict:
                    feature_groups_dict[group_name] = {
                        "name": group_name,
                        "features": []
                    }
                options = json.loads(options_str) if options_str else None
                feature_details = {
                    "name": feature_name,
                    "description": description,
                    "type": f_type,
                    "value": value,
                    "min": min_val,
                    "max": max_val,
                    "options": options,
                    "representation": representation
                }
                feature_groups_dict[group_name]["features"].append(feature_details)
            camera_data["features"] = list(feature_groups_dict.values())
            return camera_data

    except Exception as e:
        logger.error(f"Error fetching camera '{camera_identifier}' with features: {e}")
        return None
    finally:
        if conn:
            conn.close()

# --- Preset Management Functions ---

def create_preset(device_identifier: str, name: str, configuration: dict) -> bool:
    """Creates a new preset in the database."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot create preset.")
        return False
    try:
        with conn.cursor() as cur:
            # Ensure configuration is stored as JSONB
            config_json = json.dumps(configuration)
            cur.execute(
                "INSERT INTO presets (device_identifier, name, configuration) VALUES (%s, %s, %s) ON CONFLICT (device_identifier, name) DO NOTHING;",
                (device_identifier, name, config_json)
            )
            conn.commit()
            logger.info(f"Preset '{name}' created for device '{device_identifier}'.")
            return True
    except Exception as e:
        logger.error(f"Error creating preset '{name}' for device '{device_identifier}': {e}")
        conn.rollback()
        return False
    finally:
        if conn:
            conn.close()

async def get_preset(device_identifier: str, name: str):
    """Retrieves a specific preset by device identifier and name."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot retrieve preset.")
        return None
    try:
        with conn.cursor() as cur:
            cur.execute(
                "SELECT configuration FROM presets WHERE device_identifier = %s AND name = %s;",
                (device_identifier, name)
            )
            result = cur.fetchone()
            if result:
                # Return configuration as a Python dictionary
                return json.loads(result[0])
            else:
                logger.warning(f"Preset '{name}' not found for device '{device_identifier}'.")
                return None
    except Exception as e:
        logger.error(f"Error retrieving preset '{name}' for device '{device_identifier}': {e}")
        return None
    finally:
        if conn:
            conn.close()

async def get_presets_for_device(device_identifier: str) -> list:
    """Retrieves all presets for a given device identifier."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot retrieve presets for device.")
        return []
    try:
        with conn.cursor() as cur:
            cur.execute(
                "SELECT name, configuration FROM presets WHERE device_identifier = %s;",
                (device_identifier,)
            )
            presets = []
            rows = cur.fetchall()
            for row in rows:
                preset_name, config_json = row
                presets.append({
                    "name": preset_name,
                    "configuration": json.loads(config_json)
                })
            logger.info(f"Retrieved {len(presets)} presets for device '{device_identifier}'.")
            return presets
    except Exception as e:
        logger.error(f"Error retrieving presets for device '{device_identifier}': {e}")
        return []
    finally:
        if conn:
            conn.close()

def update_preset(device_identifier: str, name: str, new_configuration: dict) -> bool:
    """Updates an existing preset's configuration."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot update preset.")
        return False
    try:
        with conn.cursor() as cur:
            config_json = json.dumps(new_configuration)
            cur.execute(
                "UPDATE presets SET configuration = %s WHERE device_identifier = %s AND name = %s;",
                (config_json, device_identifier, name)
            )
            conn.commit()
            if cur.rowcount == 0:
                logger.warning(f"Preset '{name}' not found for device '{device_identifier}' during update.")
                return False
            logger.info(f"Preset '{name}' updated for device '{device_identifier}'.")
            return True
    except Exception as e:
        logger.error(f"Error updating preset '{name}' for device '{device_identifier}': {e}")
        conn.rollback()
        return False
    finally:
        if conn:
            conn.close()

def delete_preset(device_identifier: str, name: str) -> bool:
    """Deletes a preset from the database."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot delete preset.")
        return False
    try:
        with conn.cursor() as cur:
            cur.execute(
                "DELETE FROM presets WHERE device_identifier = %s AND name = %s;",
                (device_identifier, name)
            )
            conn.commit()
            if cur.rowcount == 0:
                logger.warning(f"Preset '{name}' not found for device '{device_identifier}' during deletion.")
                return False
            logger.info(f"Preset '{name}' deleted for device '{device_identifier}'.")
            return True
    except Exception as e:
        logger.error(f"Error deleting preset '{name}' for device '{device_identifier}': {e}")
        conn.rollback()
        return False
    finally:
        if conn:
            conn.close()

# --- END Preset Management Functions ---
