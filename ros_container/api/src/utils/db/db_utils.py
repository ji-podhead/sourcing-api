#!/usr/bin/env python3
import gi
import sys
import os
import json
import psycopg2
import logging
from typing import Union
import ipaddress

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


# Database configuration
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
    except Exception as e:
        logger.error(f"Error updating feature '{feature_name}' in group '{group_name}': {e}")
        conn.rollback()

def update_feature_writability(conn, feature_name, group_name, is_writable):
    """Updates a specific feature's writability in the database."""
    try:
        with conn.cursor() as cur:
            cur.execute("""
                UPDATE features
                SET is_writable = %s
                WHERE name = %s AND group_id = (SELECT id FROM feature_groups WHERE name = %s);
            """, (is_writable, feature_name, group_name))
            conn.commit()
    except Exception as e:
        logger.error(f"Error updating writability for feature '{feature_name}' in group '{group_name}': {e}")
        conn.rollback()

async def create_gigE_camera(camera_name: str, camera_ip: str, details: dict):
    """Creates a new camera entry and populates its features in the database.
    Returns the camera ID."""
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
            return camera_id
    except Exception as e:
        logger.error(f"An unexpected error occurred during camera creation/initialization: {e}")
        if conn: conn.rollback()
        return None
    finally:
        if conn: conn.close()

async def get_all_gigE_cams():
    """Retrieves all cameras from the database."""
    conn = get_db_connection()
    if not conn:
        return []
    cameras = []
    try:
        with conn.cursor() as cur:
            cur.execute("SELECT id, camera_name, camera_ip FROM cameras;")
            rows = cur.fetchall()
            for row in rows:
                cameras.append({"id": row[0], "camera_name": row[1], "camera_ip": row[2]})
        return cameras
    except Exception as e:
        logger.error(f"Error retrieving cameras: {e}")
        return []
    finally:
        if conn:
            conn.close()

async def get_gigE_camera(camera_id: int):
    """Fetches camera details and its features from the database by ID."""
    conn = get_db_connection()
    if not conn:
        return None
    try:
        with conn.cursor() as cur:
            cur.execute("SELECT id, camera_name, camera_ip, type, config, user_notes, publishing_preset FROM cameras WHERE id = %s;", (camera_id,))
            camera_row = cur.fetchone()
            if not camera_row:
                return None
            camera_data = {
                "id": camera_row[0], "camera_name": camera_row[1], "camera_ip": camera_row[2],
                "type": camera_row[3], "config": camera_row[4], "user_notes": camera_row[5],
                "publishing_preset": camera_row[6], "status": "stopped", "features": []
            }
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
                    f.representation,
                    f.is_writable
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
                group_name, feature_name, description, f_type, value, min_val, max_val, options_str, representation, is_writable = row
                if group_name not in feature_groups_dict:
                    feature_groups_dict[group_name] = {
                        "name": group_name,
                        "features": []
                    }
                options = json.loads(options_str) if options_str else None
                feature_details = {
                    "name": feature_name, "description": description, "type": f_type, "value": value,
                    "min": min_val, "max": max_val, "options": options, "representation": representation,
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
    conn = get_db_connection()
    if not conn: return False
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
        if conn: conn.close()

def update_camera_publishing_preset(camera_id: int, preset_name: str) -> bool:
    conn = get_db_connection()
    if not conn: return False
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
        if conn: conn.close()

def delete_camera_from_db(camera_id: int) -> bool:
    """Deletes a camera and its associated data from the database."""
    conn = get_db_connection()
    if not conn: return False
    try:
        with conn.cursor() as cur:
            camera_name = get_camera_name_by_id(camera_id)
            cur.execute("DELETE FROM presets WHERE camera_name = %s;", (camera_name,))
            cur.execute("DELETE FROM features WHERE group_id IN (SELECT id FROM feature_groups WHERE name LIKE %s);", (f"%{camera_name}%",))
            cur.execute("DELETE FROM cameras WHERE id = %s;", (camera_id,))
            conn.commit()
            return cur.rowcount > 0
    except Exception as e:
        logger.error(f"Error deleting camera ID '{camera_id}': {e}")
        conn.rollback()
        return False
    finally:
        if conn: conn.close()

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
        if conn: conn.close()

async def get_presets_for_device(camera_id: int) -> list:
    conn = get_db_connection()
    if not conn: return []
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
        if conn: conn.close()

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
