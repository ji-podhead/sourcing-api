import roslibpy
import asyncio
import logging
import threading
import base64
import time
import sys
from typing import Dict, Any, Tuple, Optional

# GObject-Bibliotheken importieren
import gi
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis, GLib
from utils.db.db_utils import get_gigE_camera

logger = logging.getLogger(__name__)

import asyncio

class GigECameraNode:
    def __init__(self, camera_id: str, ros_host: str = 'localhost', ros_port: int = 9090):
        self.camera_id = camera_id
        print(f"Initializing GigECameraNode for camera ID: {camera_id}")
        self.is_running = False
        try:
            self.cam = asyncio.get_event_loop().run_until_complete(get_gigE_camera(camera_id))
        except Exception as e:
            logger.error(f"Error initializing camera {camera_id}: {e}")
            self.cam = None

        if self.cam is None:
            raise ValueError(f"No camera found with ID '{camera_id}' in the database.")
        
        self.camera_identifier = self.cam['identifier']
        logger.info(f"[{self.camera_id}] Initializing GigECameraNode for camera identifier: {self.camera_identifier}")
        self._shutdown_lock = threading.Lock()
        
        self._ros_connected_event = threading.Event()
        self._ros_connection_failed = False

        self.ros_client = roslibpy.Ros(host=ros_host, port=ros_port)
        self.ros_client.on_ready(self._on_ros_connect)

        self.set_feature_service_name = f"/gige_camera_{self.camera_id.replace('.', '_').replace('-', '_')}/set_feature"
        self.image_topic_name = f"/gige_camera_{self.camera_id.replace('.', '_').replace('-', '_')}/image_raw"
        
        self.set_feature_service: Optional[roslibpy.Service] = None
        self.image_pub: Optional[roslibpy.Topic] = None

        self.camera: Optional[Aravis.Camera] = None
        self.stream: Optional[Aravis.Stream] = None
        self.width = 0
        self.height = 0
        self.pixel_format_str = ""

        self.glib_loop: Optional[GLib.MainLoop] = None
        self._glib_thread: Optional[threading.Thread] = None
        self._ros_thread: Optional[threading.Thread] = None

    def _on_ros_connect(self):
        """Callback, der ausgeführt wird, wenn die ROS-Verbindung bereit ist."""
        logger.info(f"[{self.camera_id}] Successfully connected to rosbridge.")
        print(self.items())
        self._ros_connected_event.set()

        try:
            self.set_feature_service = roslibpy.Service(self.ros_client, self.set_feature_service_name, 'interfaces/srv/SetFeature')
            self.image_pub = roslibpy.Topic(self.ros_client, self.image_topic_name, 'sensor_msgs/msg/Image')

            self._glib_thread = threading.Thread(target=self._run_glib_loop, daemon=True)
            self._glib_thread.start()
            logger.info(f"[{self.camera_id}] GLib event loop thread started.")
            
            self._initialize_camera()
            
            self.is_running = True
            logger.info(f"[{self.camera_id}] Camera node is now fully running.")

        except Exception as e:
            logger.error(f"[{self.camera_id}] Failed during camera startup after ROS connection: {e}", exc_info=True)
            self.is_running = False
            # **ÄNDERUNG: Rufe terminate() nicht aus dem Callback auf, um den Absturz zu vermeiden**
            # Der ROS-Thread wird von selbst enden, wenn die Verbindung abbricht.
            # Dies verhindert den 'TwistedEventLoopManager' object has no attribute '_thread' Fehler.
            logger.error(f"[{self.camera_id}] Shutting down due to camera initialization failure.")


    def _initialize_camera(self):
        """Initialisiert die Aravis-Kamera."""
        logger.info(f"[{self.camera_id}] Searching for Aravis camera with ID: {self.camera_identifier}...")
        
        self.camera = Aravis.Camera.new(self.camera_identifier)
        if self.camera is None:
            raise Exception(f"No Aravis camera found with ID '{self.camera_identifier}'. Use `arv-tool-0.8 -l` to find the correct ID.")
        
        logger.info(f"[{self.camera_identifier}] Camera found: {self.camera.get_model_name()} ({self.camera.get_device_id()})")
        
        self.pixel_format_str = self.camera.get_pixel_format_as_string()
        _, _, self.width, self.height = self.camera.get_region()
        logger.info(f"[{self.camera_identifier}] Resolution: {self.width}x{self.height}, PixelFormat: {self.pixel_format_str}")

        self.stream = self.camera.create_stream(None, None)
        if self.stream is None: raise Exception("Could not create stream!")
        
        payload = self.camera.get_payload()
        for _ in range(10): self.stream.push_buffer(Aravis.Buffer.new_allocate(payload))
            
        self.stream.connect('new-buffer', self.new_buffer_callback)
        self.camera.start_acquisition()
        logger.info(f'[{self.camera_id}] Camera acquisition started.')

    def _run_glib_loop(self):
        """Führt die GLib-Main-Loop für Kamera-Events aus."""
        self.glib_loop = GLib.MainLoop()
        try:
            self.glib_loop.run()
        finally:
            logger.info(f"[{self.camera_id}] GLib event loop stopped.")

    def _run_ros_loop(self):
        """Führt die ROS-Client-Loop aus und behandelt Verbindungsfehler."""
        logger.info(f"[{self.camera_id}] ROS client thread started, attempting connection...")
        try:
            self.ros_client.run_forever()
        except Exception as e:
            logger.error(f"[{self.camera_id}] ROS client loop failed to start or crashed.", exc_info=True)
            self._ros_connection_failed = True
        finally:
            logger.info(f"[{self.camera_id}] ROS client loop has terminated.")
            self._ros_connected_event.set()

    def start(self) -> bool:
        """
        Startet den Node und wartet auf eine erfolgreiche ROS-Verbindung.
        """
        if self.is_running:
            logger.warning(f"[{self.camera_id}] Start called but already running.")
            return True

        self._ros_connected_event.clear()
        self._ros_connection_failed = False
        
        logger.info(f"[{self.camera_id}] Initiating connection to rosbridge...")
        self._ros_thread = threading.Thread(target=self._run_ros_loop, daemon=True)
        self._ros_thread.start()

        is_connected = self._ros_connected_event.wait(timeout=10.0)

        if self._ros_connection_failed or not is_connected or not self.ros_client.is_connected:
            logger.error(f"[{self.camera_id}] Failed to connect to rosbridge.")
            return False
        
        logger.info(f"[{self.camera_id}] ROS connection established. Startup proceeding.")
        return True

    def shutdown(self):
        """Fährt den Node sicher herunter."""
        with self._shutdown_lock:
            if not self.is_running and not self.ros_client.is_connected:
                return
            logger.info(f"[{self.camera_id}] Shutting down camera node...")
            self.is_running = False

            if self.camera:
                try: self.camera.stop_acquisition()
                except GLib.Error as e: logger.warning(f"[{self.camera_id}] Error stopping camera acquisition: {e.message}")

            if self.glib_loop and self.glib_loop.is_running(): self.glib_loop.quit()
            
            if self.ros_client.is_connected: self.ros_client.terminate()

        current_thread = threading.current_thread()
        if self._glib_thread and self._glib_thread.is_alive() and self._glib_thread != current_thread:
            self._glib_thread.join(timeout=2)
        if self._ros_thread and self._ros_thread.is_alive() and self._ros_thread != current_thread:
             self._ros_thread.join(timeout=2)
        logger.info(f"[{self.camera_id}] Shutdown complete.")
    def new_buffer_callback(self, stream):
        """Verarbeitet neue Kamerabilder."""
        if not self.is_running:
            return

        buffer = stream.try_pop_buffer()
        if buffer is None: return

        try:
            if buffer.get_status() != Aravis.BufferStatus.SUCCESS: return
            buffer_data = buffer.get_data()
            if not buffer_data: return

            now = time.time()
            secs = int(now)
            nsecs = int((now - secs) * 1e9)
            
            is_mono16 = "16" in self.pixel_format_str
            bytes_per_pixel = 2 if is_mono16 else 1
            encoding = "mono16" if is_mono16 else "mono8"

            ros_msg = {
                'header': {'stamp': {'sec': secs, 'nanosec': nsecs}, 'frame_id': "camera_link"},
                'height': self.height, 'width': self.width, 'encoding': encoding,
                'is_bigendian': sys.byteorder == 'big',
                'step': self.width * bytes_per_pixel,
                'data': base64.b64encode(buffer_data).decode('utf-8')
            }
            
            if self.image_pub and self.ros_client.is_connected:
                self.image_pub.publish(roslibpy.Message(ros_msg))
        finally:
            stream.push_buffer(buffer)

    # Async-Funktionen bleiben unverändert
    async def set_feature(self, feature_name: str, value: Any) -> Tuple[bool, str]:
        if not self.ros_client.is_connected or self.set_feature_service is None:
            return False, "Not connected to ROS or service not ready"

        request = roslibpy.ServiceRequest({'feature_name': str(feature_name), 'value': str(value)})
        try:
            loop = asyncio.get_running_loop()
            response = await loop.run_in_executor(None, lambda: self.set_feature_service.call(request, timeout=5.0))
            if response is None: raise Exception("Service call timed out.")
            return response['success'], response.get('message', '')
        except Exception as e:
            logger.error(f"[{self.camera_id}] Error calling service: {e}", exc_info=True)
            return False, str(e)

    async def get_features(self) -> Dict[str, Any]:
        if not self.is_running: return {}
        return {"PixelFormat": self.pixel_format_str, "Width": self.width, "Height": self.height}
