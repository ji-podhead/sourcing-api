import roslibpy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import asyncio
import logging
import threading
import time
import sys
from typing import Dict, Any, Tuple, Optional
import signal
import gi
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis, GLib
from utils.db.db_utils import get_gigE_camera
import numpy as np
from multiprocessing import Queue
import queue

logger = logging.getLogger(__name__)

class Ros2PublisherNode(Node):
    def __init__(self, camera_id):
        super().__init__(f'ros2_publisher_node_{camera_id}')
        
        # Parameter deklarieren
        self.declare_parameter('camera_name', '')
        self.declare_parameter('pixel_format', 'mono8')
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)
        self.name= f"/gige_camera_{str(camera_id).replace('.', '_').replace('-', '_')}"
        self.services_publisher = self.create_publisher(String, 
           self.name+"/services", 
            10)
                
        self.image_publisher = self.create_publisher(Image, self.name+"/image_raw", 10)
        self.get_logger().info('✓ Bild-Publisher erstellt')   
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.pixel_format = self.get_parameter('pixel_format').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.timer = self.create_timer(1.0, self.timer_callback)

    def normalize_frame_16bit(self, gray_image: np.ndarray) -> Optional[np.ndarray]:
        """Normalizes a grayscale image to uint8."""
        if gray_image is None:
            return None
        min_val = np.min(gray_image)
        max_val = np.max(gray_image)
        if max_val == min_val:
            logger.warning(f"[{self.camera_id}] min_val equals max_val during normalization. Skipping frame.")
            return None
        
        # Ensure calculations are done with float to avoid overflow/precision issues
        normalized = (gray_image.astype(np.float32) - min_val) / (max_val - min_val)
        frame_8bit = (normalized * 255.0).astype(np.uint8)
        return frame_8bit

    def timer_callback(self):
        # Parameter aktualisieren und veröffentlichen
        features = {
            'camera_name': self.camera_name,
            'pixel_format': self.pixel_format,
            'width': self.width,
            'height': self.height
        }
        
        msg = String()
        msg.data = str(features)
        self.services_publisher.publish(msg)
        self.get_logger().debug(f'Published features: {msg.data}')
        
class GigECameraNode:
    def __init__(self, camera_id: int, command_queue: Queue, ros2_node: Node, ros_host: str = 'localhost', ros_port: int = 9090):
        self.shutdown_signal_received = False
        signal.signal(signal.SIGTERM, self._handle_shutdown_signal)
        signal.signal(signal.SIGINT, self._handle_shutdown_signal)
        
        self.camera_id = camera_id
        self.command_queue = command_queue
        self.ros2_node = ros2_node
        self.rclpy_image_publisher = ros2_node.image_publisher # Get the rclpy image publisher
        logger.info(f"[{self.camera_id}] Initializing GigECameraNode for camera ID: {camera_id}")
        
        self.is_running = False
        self.cam = get_gigE_camera(camera_id)
        if self.cam is None:
            raise ValueError(f"No camera found with ID '{camera_id}' in the database.")
        
        self.camera_identifier = self.cam['camera_name']
        logger.info(f"[{self.camera_id}] Initializing GigECameraNode for camera identifier: {self.camera_identifier}")
        
        # ROS Client mit Thread-Sicherheit
        self.ros_client = roslibpy.Ros(host=ros_host, port=ros_port)
        self.ros_client.on_ready(self._on_ros_connect)
        
        # Topic-Namen erstellen
        self.set_feature_service_name = f"/gige_camera_{str(self.camera_id).replace('.', '_').replace('-', '_')}/set_feature"
        self.image_topic_name = f"/gige_camera_{str(self.camera_id).replace('.', '_').replace('-', '_')}/image_raw"
        self.services_topic_name = f"/gige_camera_{str(self.camera_id).replace('.', '_').replace('-', '_')}/services"
        
        # Optionalen initialisieren
        self.set_feature_service: Optional[roslibpy.Service] = None
        self.image_pub: Optional[roslibpy.Topic] = None
        self.services_pub: Optional[roslibpy.Topic] = None
        self.camera: Optional[Aravis.Camera] = None
        self.stream: Optional[Aravis.Stream] = None
        self.width = 0
        self.height = 0
        self.pixel_format_str = ""
        # REMOVED: self.glib_loop: Optional[GLib.MainLoop] = None

    def _handle_shutdown_signal(self, signum, frame):
        """Behandelt SIGTERM und SIGINT Signale für kontrollierten Shutdown."""
        logger.info(f"[{self.camera_id}] Received signal {signal.Signals(signum).name}. Triggering shutdown.")
        self.shutdown_signal_received = True
        self.command_queue.put('shutdown')

    def _on_ros_connect(self):
        """Callback, der ausgeführt wird, wenn die ROS-Verbindung bereit ist."""
        logger.info(f"[{self.camera_id}] Successfully connected to rosbridge.")
        try:
            # ROS-Client mit Thread-Sicherheit
            self.ros_client.on_close(self._on_ros_disconnect) # KEEP THIS
            self.ros_client.on_error(self._on_ros_error) # KEEP THIS
            
            # Topic mit höherer Queue-Size für bessere Performance
            self.image_pub = roslibpy.Topic(
                self.ros_client,
                self.image_topic_name,
                'sensor_msgs/msg/Image',
                queue_size=10
            )
            
            self.services_pub = roslibpy.Topic(
                self.ros_client,
                self.services_topic_name,
                'std_msgs/String',
                queue_size=10
            )

            # Test-Topic für Statusmeldungen
            self.status_pub = roslibpy.Topic(
                self.ros_client,
                f'/gige_camera_{self.camera_id}/status',
                'std_msgs/msg/String',
                queue_size=5
            )
            
            # Initialisierung bestätigen
            self._publish_status("Camera initialized successfully")
            
        except Exception as e:
            logger.error(f"[{self.camera_id}] Failed during camera startup after ROS connection: {e}",
                        exc_info=True)
            self.is_running = False

    def _publish_status(self, message: str):
        """Veröffentlicht einen Statusbericht über das Test-Topic."""
        try:
            status_msg = roslibpy.Message({
                'data': f"{message} at {time.time()}"
            })
            self.status_pub.publish(status_msg)
            logger.debug(f"[{self.camera_id}] Status published: {message}")
        except Exception as e:
            logger.error(f"[{self.camera_id}] Error publishing status: {e}")

    def _initialize_camera(self):
        """Initialisiert die Aravis-Kamera mit korrekter Thread-Synchronisation."""
        logger.info(f"[{self.camera_id}] Searching for Aravis camera with ID: {self.camera_identifier}...")
        self.camera = Aravis.Camera.new(self.camera_identifier)
        if self.camera is None:
            raise Exception(f"No Aravis camera found with ID '{self.camera_identifier}'. Use `arv-tool-0.8 -l` to find the correct ID.")
        
        # Changed from (None, self) to (None, None) as _on_new_buffer is removed and no callback is intended.
        self.stream = self.camera.create_stream(None, None) 
        payload = self.camera.get_payload()
        # Mehrere Puffer für bessere Performance
        for i in range(10):
            self.stream.push_buffer(Aravis.Buffer.new_allocate(payload))
        
        # REMOVED: self.stream.connect('new-buffer', self._on_new_buffer) 
        logger.info(f"[{self.camera_id}] Stream created and buffers allocated.")

        self.camera.start_acquisition()
        logger.info(f"[{self.camera_id}] Camera acquisition started.")

    # NEW METHOD FROM SNIPPET
    def _start_roslibpy_thread(self):
        """Startet den roslibpy Client in einem separaten Thread."""
        logger.info(f"[{self.camera_id}] Starting roslibpy client thread...")
        self.roslibpy_thread = threading.Thread(target=self.ros_client.run_forever)
        self.roslibpy_thread.daemon = True # Thread beendet sich mit dem Hauptprogramm
        self.roslibpy_thread.start()

    async def start(self): # REPLACE ENTIRE METHOD BODY
        """Startet die Kamera-Erfassung und die Hauptschleife."""
        if self.is_running:
            logger.warning(f"[{self.camera_id}] Start called but already running.")
            return

        logger.info(f"[{self.camera_id}] Starting GigECameraNode...")

        try:
            # 1. Kamera initialisieren
            if not self._initialize_camera():
                return
            
            # 2. Roslibpy-Client in einem Hintergrund-Thread starten
            self._start_roslibpy_thread()

            # --- NEU: Warten, bis der ROS-Client verbunden ist ---
            max_wait_time = 10 # seconds
            start_time = time.time()
            while not self.ros_client.is_connected and (time.time() - start_time) < max_wait_time:
                time.sleep(0.1)
            
            if not self.ros_client.is_connected:
                logger.error(f"[{self.camera_id}] Failed to connect to ROS within {max_wait_time} seconds.")
                self._publish_status("CRITICAL: Failed to connect to ROS.")
                return # Exit if not connected

            # 3. Kamera-Akquise starten
            self.camera.start_acquisition()
            logger.info(f"[{self.camera_id}] Camera acquisition started. Entering main loop.")
            
            self.is_running = True
            
            # 4. Hauptschleife
            while True:
                # Auf Befehle in der Queue prüfen
                self._check_command_queue()
                if not self.is_running: # erneut prüfen, falls Befehl 'shutdown' war
                    logger.info("shutting down mainloop because it was cancled by command queue")
                    break

                # Auf einen neuen Buffer warten (blockiert, mit Timeout von 1 Sekunde)
                buffer = self.stream.pop_buffer(1000)

                if buffer:
                    if buffer.get_status() == Aravis.BufferStatus.SUCCESS:
                        # Bilddaten verarbeiten
                        payload = buffer.get_data()
                        
                        # Annahme: Kamera liefert 16bit, wir normalisieren zu 8bit
                        # Passe dtype und Normalisierung an dein tatsächliches Kameraformat an!
                        frame_16bit = np.ndarray(buffer=payload, dtype=np.uint16, shape=(self.height, self.width))
                        
                        # Normalisierung
                        frame_8bit = self.ros2_node.normalize_frame_16bit(frame_16bit)

                        if frame_8bit is not None:
                            # Bild über rclpy publishen
                            msg = Image()
                            msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
                            msg.height = self.height
                            msg.width = self.width
                            msg.encoding = 'mono8'
                            msg.is_bigendian = False
                            msg.step = self.width
                            msg.data = frame_8bit.tobytes()
                    
                            self.rclpy_image_publisher.publish(msg)
                        else:
                            logger.warning("failed to get frame: normalized frame is none")
                    else:
                        logger.warning("failed to get frame: no aravis buffer success")
                    # WICHTIG: Buffer immer zurück in den Stream pushen!
                    self.stream.push_buffer(buffer)
                else:
                    logger.warning("failed to get frame: no buffer")    
                # Gib rclpy Zeit, um seine eigenen Callbacks (z.B. Timer) zu verarbeiten
                rclpy.spin_once(self.ros2_node, timeout_sec=0.1)

        except Exception as e:
            logger.error(f"[{self.camera_id}] Critical error in camera process: {e}", exc_info=True)
        finally:
            logger.info(f"[{self.camera_id}] Exiting main loop.")
            self.shutdown()

    def shutdown(self): # Adjust to fit new loop structure
        """Fährt den Node sicher herunter."""
        logger.info(f"[{self.camera_id}] Shutting down camera node...")
        self.is_running = False # Already set in loop break or signal handler

        # Kamera stoppen
        if self.camera:
            try:
                self.camera.stop_acquisition()
                logger.info(f"[{self.camera_id}] Camera acquisition stopped.")
            except GLib.Error as e: # Keep this exception handling
                logger.warning(f"[{self.camera_id}] Error stopping camera acquisition: {e.message}")

        # ROS Client terminieren
        if self.ros_client and self.ros_client.is_connected:
            self.ros_client.terminate()
            logger.info(f"[{self.camera_id}] Roslibpy client terminated.")
        
        # REMOVED: GLib Loop cleanup as GLib loop is removed.
        # if self.glib_loop and self.glib_loop.is_running():
        #     self.glib_loop.quit()

        logger.info(f"[{self.camera_id}] Shutdown sequence complete.")

    # KEEP THESE METHODS FROM ORIGINAL FILE
    def _on_ros_disconnect(self, event): # KEEP THIS
        """Callback wenn die ROS-Verbindung getrennt wird."""
        logger.warning(f"[{self.camera_id}] ROS connection disconnected: {event}")
        self.is_running = False
        # REMOVED: GLib Loop cleanup

    def _on_ros_error(self, error): # KEEP THIS
        """Callback wenn ein ROS-Fehler auftritt."""
        logger.error(f"[{self.camera_id}] ROS error occurred: {error}")
        self.is_running = False
        # REMOVED: GLib Loop cleanup

    async def set_feature(self, feature_name: str, value: Any) -> Tuple[bool, str]: # KEEP THIS
        """Setzt eine Kameraparameter über den ROS-Service."""
        if not self.ros_client.is_connected or self.set_feature_service is None:
            return False, "Not connected to ROS or service not ready"
        
        request = roslibpy.ServiceRequest({'feature_name': str(feature_name), 'value': str(value)})
        try:
            loop = asyncio.get_running_loop()
            response = await loop.run_in_executor(None, lambda: self.set_feature_service.call(request, timeout=5.0))
            if response is None:
                raise Exception("Service call timed out.")
            return response['success'], response.get('message', '')
        except Exception as e:
            logger.error(f"[{self.camera_id}] Error calling service: {e}", exc_info=True)
            return False, str(e)

    async def get_features(self) -> Dict[str, Any]: # KEEP THIS
        """Gibt die aktuellen Kameraparameter zurück."""
        if not self.is_running:
            return {}
        return {"PixelFormat": self.pixel_format_str, "Width": self.width, "Height": self.height}

def run_camera_process(camera_id: int, command_queue: Queue):
    """This function is the entry point for the camera driver subprocess."""
    logger.info(f"[Process for camera {camera_id}] Starting...")
    ros2_node = None
    try:
        rclpy.init()
        ros2_node = Ros2PublisherNode(camera_id)
        
        # Den GigECameraNode direkt erstellen und starten
        gige_node = GigECameraNode(camera_id=camera_id, command_queue=command_queue, ros2_node=ros2_node)
        asyncio.run(gige_node.start()) # Diese Methode enthält jetzt die Hauptschleife und blockiert bis zum Ende

    except Exception as e:
        logger.error(f"[Process for camera {camera_id}] An unhandled exception occurred: {e}", exc_info=True)
    finally:
        if rclpy.ok():
            if ros2_node:
                ros2_node.destroy_node()
            rclpy.shutdown()
        logger.info(f"[Process for camera {camera_id}] Terminated.")
