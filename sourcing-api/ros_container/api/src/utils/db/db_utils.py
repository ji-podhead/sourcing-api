#!/usr/bin/env python3
import gi
import sys
import os
import json
import psycopg2
import logging
import ipaddress

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

def get_camera_id_by_name(camera_name: str) -> int:
    """Retrieves the camera ID for a specific camera name."""
    conn = get_db_connection()
    if not conn:
        return -1
    try:
        with conn.cursor() as cur:
            cur.execute("SELECT id FROM cameras WHERE camera_name = %s;", (camera_name,))
            result = cur.fetchone()
            return result[0] if result else -1
    except Exception as e:
        logger.error(f"Error getting camera id for name {camera_name}: {e}")
        return -1
    finally:
        if conn:
            conn.close()

def get_camera_name_by_id(camera_id: int) -> str:
    """Retrieves the camera name for a specific camera ID."""        
    conn = get_db_connection()
    if not conn:
       return ""
    try:
        with conn.cursor() as cur:
            cur.execute("SELECT camera_name FROM cameras WHERE id = %s;", (camera_id,))
            result = cur.fetchone()
            return result[0] if result else ""
    except Exception as e:
        logger.error(f"Error getting camera name for id {camera_id}: {e}")
        return ""
    finally:
        if conn:
            conn.close()

def get_camera_ip_by_id(camera_id: int) -> str:
    """Retrieves the camera IP for a specific camera ID."""
    conn = get_db_connection()
    if not conn:
        return ""
    try:
        with conn.cursor() as cur:
            cur.execute("SELECT camera_ip FROM cameras WHERE id = %s;", (camera_id,))
            result = cur.fetchone()
            return result[0] if result else ""
    except Exception as e:
        logger.error(f"Error getting camera ip for id {camera_id}: {e}")
        return ""
    finally:
        if conn:
            conn.close()

def is_valid_ip(ip: str):
    try:
        ipaddress.ip_address(ip)
        return True
    except ValueError:
        return False
    
    
async def find_camera_by_identifier(identifier: str):
    """Finds a camera by its name or IP address."""
    conn = get_db_connection()
    if not conn:
        return None
    try:
        with conn.cursor() as cur:
            if is_valid_ip(identifier):
                cur.execute("SELECT id FROM cameras WHERE camera_ip = %s;", (identifier,))
            else:
                cur.execute("SELECT id FROM cameras WHERE camera_name = %s;", (identifier,))
            result = cur.fetchone()
            return result[0] if result else None
    except Exception as e:
        logger.error(f"Error finding camera by identifier {identifier}: {e}")
        return None
    finally:
        if conn:
            conn.close()

async def gigE_camera_exists(camera_name: str) -> bool:
    """Checks if a camera with the given name exists in the database."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot check camera existence.")
        return False
    try:
        with conn.cursor() as cur:
            cur.execute("SELECT COUNT(*) FROM cameras WHERE camera_name = %s;", (camera_name,))
            count = cur.fetchone()[0]
            return count > 0
    except Exception as e:
        logger.error(f"Error checking existence of camera '{camera_name}': {e}")
        if conn:
            conn.close()
        return False
    
def get_gigE_features_from_db(conn, camera_id: int):
    """Fetches feature names and types from the database for a specific camera."""
    features_to_update = []
    try:
        with conn.cursor() as cur:
            cur.execute("""
                SELECT f.name, f.type, fg.group_name
                FROM features f
                JOIN feature_groups fg ON f.group_id = fg.id
                WHERE fg.camera_id = %s;
            """, (camera_id,))
            for row in cur.fetchall():
                feature_name, feature_type, group_name = row
                features_to_update.append({
                    "name": feature_name,
                    "type": feature_type,
                    "group_name": group_name
                })
        logger.info(f"Fetched {len(features_to_update)} features from database for camera '{camera_id}'.")
        return features_to_update
    except Exception as e:
        logger.error(f"Error fetching features for camera '{camera_id}' from database: {e}")
        return []

def update_gigE_feature_in_db(conn, camera_id: int, feature_name: str, group_name: str, new_value: str):
    """Updates a specific feature's value in the database."""
    try:
        with conn.cursor() as cur:
            cur.execute("""
                UPDATE features
                SET value = %s
                WHERE name = %s AND group_id = (SELECT id FROM feature_groups WHERE camera_id = %s AND group_name = %s);
            """, (new_value, feature_name, camera_id, group_name))
            conn.commit()
    except Exception as e:
        logger.error(f"Error updating feature '{feature_name}' in group '{group_name}': {e}")
        conn.rollback()

def update_feature_writability(conn, camera_id: int, feature_name: str, group_name: str, is_writable: bool):
    """Updates a specific feature's writability in the database."""
    try:
        with conn.cursor() as cur:
            cur.execute("""
                UPDATE features
                SET is_writable = %s
                WHERE name = %s AND group_id = (SELECT id FROM feature_groups WHERE camera_id = %s AND group_name = %s);
            """, (is_writable, feature_name, camera_id, group_name))
            conn.commit()
    except Exception as e:
        logger.error(f"Error updating writability for feature '{feature_name}' in group '{group_name}': {e}")
        conn.rollback()

async def create_gigE_camera(camera_name: str, camera_ip: str, details: dict):
    """Creates a new camera entry and populates its features in the database. Returns the camera ID."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot create camera.")
        return None

    try:
        with conn.cursor() as cur:
            cur.execute(
                "INSERT INTO cameras (camera_name, camera_ip, type, config) VALUES (%s, %s, %s, %s) ON CONFLICT (camera_name) DO UPDATE SET camera_ip = EXCLUDED.camera_ip RETURNING id;",
                (camera_name, camera_ip, "gigE", json.dumps({"protocol": "gigE", "ip": camera_ip}))
            )
            camera_id = cur.fetchone()[0]
            conn.commit()
            logger.info(f"Successfully created or updated camera entry for: {camera_name} with ID: {camera_id}")

            if details:
                logger.info(f"Initializing features for camera: {camera_name}...")
                for group_name, group_details in details.items():
                    logger.info(f"Processing feature group: {group_name}")
                    cur.execute(
                        "INSERT INTO feature_groups (camera_id, group_name) VALUES (%s, %s) ON CONFLICT (camera_id, group_name) DO NOTHING RETURNING id;",
                        (camera_id, group_name)
                    )
                    group_id_row = cur.fetchone()
                    
                    if group_id_row:
                        group_id = group_id_row[0]
                    else:
                        cur.execute("SELECT id FROM feature_groups WHERE camera_id = %s AND group_name = %s;", (camera_id, group_name))
                        group_id_row = cur.fetchone()
                        if not group_id_row:
                            logger.warning(f"Could not get or create group ID for '{group_name}' with camera ID '{camera_id}'. Skipping features in this group.")
                            continue
                        group_id = group_id_row[0]

                    actual_features_dict = group_details.get("features")
                    if not actual_features_dict or not isinstance(actual_features_dict, dict):
                        logger.warning(f"Group '{group_name}' does not contain a valid 'features' dictionary. Skipping.")
                        continue

                    for feature_name, feature_details_data in actual_features_dict.items():
                        try:
                            feature_type = feature_details_data.get("type")
                            value = feature_details_data.get("value")
                            tooltip = feature_details_data.get("tooltip", "")
                            description = feature_details_data.get("description", "")

                            if isinstance(value, (dict, list)):
                                value_str = json.dumps(value)
                            else:
                                value_str = str(value)

                            options = feature_details_data.get("options")
                            options_str = json.dumps(options) if options else None
                            min_val = feature_details_data.get("min")
                            max_val = feature_details_data.get("max")
                            
                            cur.execute("""
                                INSERT INTO features (name, type, value, tooltip, description, group_id, options, min, max)
                                VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)
                                ON CONFLICT (name, group_id) DO UPDATE SET
                                    type = EXCLUDED.type,
                                    value = EXCLUDED.value,
                                    tooltip = EXCLUDED.tooltip,
                                    description = EXCLUDED.description,
                                    options = EXCLUDED.options,
                                    min = EXCLUDED.min,
                                    max = EXCLUDED.max;
                            """, (feature_name, feature_type, value_str, tooltip, description, group_id, options_str, min_val, max_val))
                        except Exception as feature_e:
                            logger.error(f"Error processing feature '{feature_name}' in group '{group_name}': {feature_e}")
                            conn.rollback()
                        else:
                            conn.commit()
        
        if details:
            logger.info(f"Creating 'Default' preset for camera '{camera_id}'...")
            default_configuration = {}
            for group_name, group_details in details.items():
                default_configuration[group_name] = {}
                if "features" in group_details:
                    for feature_name, feature_details_full in group_details["features"].items():
                        default_configuration[group_name][feature_name] = {
                            "type": feature_details_full.get("type"),
                            "value": feature_details_full.get("value")
                        }
            
            if create_preset(camera_id, "Default", default_configuration):
                logger.info(f"Successfully created 'Default' preset for camera '{camera_id}'.")
            else:
                logger.error(f"Failed to create 'Default' preset for camera '{camera_id}'.")

        logger.info(f"Feature initialization for camera '{camera_id}' completed.")
        return camera_id
    except Exception as e:
        logger.error(f"An unexpected error occurred during camera creation/initialization: {e}")
        if conn:
            conn.rollback()
        return None
    finally:
        if conn:
            conn.close()
            
async def get_all_gigE_cam_ids():
    """Retrieves all camera identifiers from the database."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot retrieve camera IDs.")
        return []

    cameras = []
    try:
        with conn.cursor() as cur:
            cur.execute("SELECT id, camera_name, camera_ip FROM cameras;")
            rows = cur.fetchall()
            for row in rows:
                cameras.append({"id": row[0], "camera_name": row[1], "camera_ip": row[2]})
        logger.info(f"Retrieved {len(cameras)} cameras from the database.")
        return cameras

    except Exception as e:
        logger.error(f"Error retrieving camera IDs: {e}")
    finally:
        if conn:
            conn.close()


def get_gigE_camera(camera_id: int):
    """Fetches camera details and its features from the database by ID."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot retrieve camera details.")
        return None

    try:
        with conn.cursor() as cur:
            cur.execute("SELECT id, camera_name, camera_ip, type, config, user_notes, publishing_preset FROM cameras WHERE id = %s;", (camera_id,))

            camera_row = cur.fetchone()

            if not camera_row:
                logger.warning(f"No camera found with identifier: {camera_id}")
                return None

            camera_data = {
                "id": camera_row[0],
                "camera_name": camera_row[1],
                "camera_ip": camera_row[2],
                "type": camera_row[3],
                "config": camera_row[4],
                "user_notes": camera_row[5],
                "publishing_preset": camera_row[6],
                "status": "stopped",
                "features": []
            }

            cur.execute("""
                SELECT
                    fg.group_name,
                    f.name AS feature_name,
                    f.description,
                    f.type,
                    f.value,
                    f.min,
                    f.max,
                    f.options,
                    f.representation,
                    f.is_writable
                FROM
                    features f
                JOIN
                    feature_groups fg ON f.group_id = fg.id
                WHERE fg.camera_id = %s
                ORDER BY
                    fg.group_name, f.name;
            """, (camera_id,))
            feature_rows = cur.fetchall()

            feature_groups_dict = {}
            for row in feature_rows:
                group_name, feature_name, description, f_type, value, min_val, max_val, options_str, representation, is_writable = row

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
                    "representation": representation,
                    "is_writable": is_writable
                }
                feature_groups_dict[group_name]["features"].append(feature_details)
            camera_data["features"] = list(feature_groups_dict.values())
            return camera_data

    except Exception as e:
        logger.error(f"Error fetching camera '{camera_id}' with features: {e}")
        return None
    finally:
        if conn:
            conn.close()

def update_camera_notes(camera_id: int, notes: str) -> bool:
    """Updates the user notes for a specific camera."""
    conn = get_db_connection()
    if not conn:
        return False
    try:
        with conn.cursor() as cur:
            cur.execute("UPDATE cameras SET user_notes = %s WHERE id = %s;", (notes, camera_id))
            conn.commit()
            return True
    except Exception as e:
        logger.error(f"Error updating notes for camera '{camera_id}': {e}")
        conn.rollback()
        return False
    finally:
        if conn:
            conn.close()

def update_camera_publishing_preset(camera_id: int, preset_name: str) -> bool:
    """Updates the publishing preset for a specific camera."""
    conn = get_db_connection()
    if not conn:
        return False
    try:
        with conn.cursor() as cur:
            cur.execute("UPDATE cameras SET publishing_preset = %s WHERE id = %s;", (preset_name, camera_id))
            conn.commit()
            return True
    except Exception as e:
        logger.error(f"Error updating publishing preset for camera '{camera_id}': {e}")
        conn.rollback()
        return False
    finally:
        if conn:
            conn.close()

# --- Preset Management Functions ---

def delete_camera_from_db(camera_id: int) -> bool:
    """Deletes a camera from the database."""
    conn = get_db_connection()
    if not conn: return False
    try:
        with conn.cursor() as cur:
            cur.execute("DELETE FROM cameras WHERE id = %s;", (camera_id,))
            conn.commit()
            return cur.rowcount > 0
    except Exception as e:
        logger.error(f"Error deleting camera ID '{camera_id}': {e}")
        conn.rollback()
        return False
    finally:
        if conn:
            conn.close()



# --- Preset Management Functions ---
def create_preset(camera_id: int, name: str, configuration: dict) -> bool:
    conn = get_db_connection()
    if not conn: return False
    try:
        with conn.cursor() as cur:
            config_json = json.dumps(configuration)
            cur.execute(
                "INSERT INTO presets (camera_name, name, configuration) VALUES ((SELECT camera_name FROM cameras WHERE id = %s), %s, %s) ON CONFLICT (camera_name, name) DO NOTHING;",
                (camera_id, name, config_json)
            )
            conn.commit()
            return True
    except Exception as e:
        logger.error(f"Error creating preset '{name}' for camera ID '{camera_id}': {e}")
        conn.rollback()
        return False
    finally:
        if conn: conn.close()
# here       
async def get_preset(camera_id: int, name: str):
    conn = get_db_connection()
    if not conn: return None
    try:
        with conn.cursor() as cur:
            cur.execute(
                "SELECT configuration FROM presets WHERE camera_name = (SELECT camera_name FROM cameras WHERE id = %s) AND name = %s;",
                (camera_id, name)
            )
            result = cur.fetchone()

            return result[0] if result else None
    except Exception as e:
        logger.error(f"Error retrieving preset '{name}' for camera ID '{camera_id}': {e}")
        return None
    finally:
        if conn:
            conn.close()

async def get_presets_for_device(camera_id: int) -> list:
    """Retrieves all presets for a given device identifier."""
    conn = get_db_connection()
    if not conn:
        logger.error("Database connection failed. Cannot retrieve presets for device.")
        return []
    try:
        with conn.cursor() as cur:
            cur.execute(
                "SELECT name, configuration FROM presets WHERE camera_name = (SELECT camera_name FROM cameras WHERE id = %s);",
                (camera_id,)
            )
            presets = [{"name": row[0], "configuration": row[1]} for row in cur.fetchall()]
            return presets
    except Exception as e:
        logger.error(f"Error retrieving presets for camera ID '{camera_id}': {e}")
        return []
    finally:
        if conn:
            conn.close()

def update_preset(camera_id: int, name: str, new_configuration: dict) -> bool:
    conn = get_db_connection()
    if not conn: return False
    try:
        with conn.cursor() as cur:
            config_json = json.dumps(new_configuration)
            cur.execute(
                "UPDATE presets SET configuration = %s WHERE camera_name = (SELECT camera_name FROM cameras WHERE id = %s) AND name = %s;",
                (config_json, camera_id, name)
            )
            conn.commit()
            return cur.rowcount > 0
    except Exception as e:
        logger.error(f"Error updating preset '{name}' for camera ID '{camera_id}': {e}")
        conn.rollback()
        return False
    finally:
        if conn: conn.close()
        
def delete_preset(camera_id: int, name: str) -> bool:
    conn = get_db_connection()
    if not conn: return False
    try:
        with conn.cursor() as cur:
            cur.execute(
                "DELETE FROM presets WHERE camera_name = (SELECT camera_name FROM cameras WHERE id = %s) AND name = %s;",
                (camera_id, name)
            )
            conn.commit()
            return cur.rowcount > 0
    except Exception as e:
        logger.error(f"Error deleting preset '{name}' for camera ID '{camera_id}': {e}")
        conn.rollback()
        return False
    finally:
        if conn: conn.close()

# --- END Preset Management Functions ---
