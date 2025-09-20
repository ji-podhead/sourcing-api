import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import logging
import threading
import time
import sys
import gi
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis
from utils.db.db_utils import get_gigE_camera
import numpy as np
from multiprocessing import Queue
import json
import queue
from queue import Empty

# Import custom services
from ros_api_package import Getfeatures, Setfeature
logger = logging.getLogger(__name__)

class GigECameraDriverNode(Node):
    def __init__(self, camera_id: int, command_queue: Queue):
        super().__init__(f'gige_camera_driver_{camera_id}')
        
        self.camera_id = camera_id
        self.command_queue = command_queue
        self.is_streaming = threading.Event()

        # Get camera info from DB
        self.cam_info = get_gigE_camera(camera_id)
        if self.cam_info is None:
            self.get_logger().error(f"No camera found with ID '{camera_id}' in the database.")
            raise ValueError(f"No camera found with ID '{camera_id}' in the database.")
        
        self.camera_identifier = self.cam_info['camera_name']
        self.get_logger().info(f"Initializing camera: {self.camera_identifier}")

        # Initialize Camera
        self.camera = Aravis.Camera.new(self.camera_identifier)
        if self.camera is None:
            self.get_logger().error(f"No Aravis camera found with ID '{self.camera_identifier}'.")
            raise Exception(f"No Aravis camera found with ID '{self.camera_identifier}'.")

        # Create Stream and add buffers
        self.stream = self.camera.create_stream(None, None)
        payload = self.camera.get_payload()
        for _ in range(10):
            self.stream.push_buffer(Aravis.Buffer.new_allocate(payload))

        # ROS2 Publishers and Services
        self.image_publisher = self.create_publisher(Image, f"/gige_camera_{camera_id}/image_raw", 10)
        self.set_feature_service = self.create_service(Setfeature, f"/gige_camera_{camera_id}/set_feature", self.set_feature_callback)
        self.get_features_service = self.create_service(Getfeatures, f"/gige_camera_{camera_id}/get_features", self.get_features_callback)

        self.get_logger().info(f"GigE Camera Driver Node for camera {camera_id} initialized.")

    def set_feature_callback(self, request, response):
        try:
            # Aravis features are case-sensitive
            self.camera.set_feature(request.feature, request.value)
            response.success = True
            response.message = f"Feature '{request.feature}' set to '{request.value}'"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to set feature '{request.feature}': {str(e)}"
            self.get_logger().error(response.message)
        return response

    def get_features_callback(self, request, response):
        try:
            # This is a simplified example. A real implementation would list and get all available features.
            features = {
                "PixelFormat": self.camera.get_feature("PixelFormat"),
                "Width": self.camera.get_feature("Width"),
                "Height": self.camera.get_feature("Height"),
                "AcquisitionFrameRate": self.camera.get_feature("AcquisitionFrameRate"),
            }
            response.features_json = json.dumps(features)
            self.get_logger().info("GetFeatures service called.")
        except Exception as e:
            response.features_json = f'{{"error": "Failed to get features: {str(e)}"}}'
            self.get_logger().error(f"Failed to get features: {str(e)}")
        return response

    def start_streaming(self):
        self.get_logger().info("Starting camera acquisition...")
        self.camera.start_acquisition()
        self.is_streaming.set()
        
        width = self.camera.get_feature("Width")
        height = self.camera.get_feature("Height")

        while self.is_streaming.is_set():
            buffer = self.stream.pop_buffer()
            if buffer:
                if buffer.get_status() == Aravis.BufferStatus.SUCCESS:
                    payload = buffer.get_data()
                    
                    # Assuming 8-bit Bayer format for simplicity, adjust if needed
                    # The original code had a 16-bit to 8-bit normalization, which might be needed
                    # depending on the actual camera pixel format.
                    frame_data = np.ndarray(buffer=payload, dtype=np.uint8, shape=(height, width)).copy()
                    
                    msg = Image()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.height = height
                    msg.width = width
                    msg.encoding = 'bayer_rggb8' # Example, should be read from camera feature
                    msg.is_bigendian = False
                    msg.step = width
                    msg.data = frame_data.tobytes()
                    self.image_publisher.publish(msg)

                self.stream.push_buffer(buffer)
        
        self.get_logger().info("Stopping camera acquisition...")
        self.camera.stop_acquisition()

    def stop_streaming(self):
        self.is_streaming.clear()

def run_camera_process(camera_id: int, command_queue: Queue):
    logger.info(f"[Process for camera {camera_id}] Starting...")
    rclpy.init()
    
    gige_node = None
    streaming_thread = None
    try:
        gige_node = GigECameraDriverNode(camera_id=camera_id, command_queue=command_queue)
        
        # Start the streaming loop in a background thread
        streaming_thread = threading.Thread(target=gige_node.start_streaming)
        streaming_thread.daemon = True
        streaming_thread.start()
        
        # Custom spin loop to handle service calls and command queue
        while rclpy.ok():
            rclpy.spin_once(gige_node, timeout_sec=0.1)
            try:
                # Check for shutdown command without blocking
                command = command_queue.get_nowait()
                if command == 'shutdown':
                    gige_node.get_logger().info("Shutdown command received, stopping.")
                    break
            except Empty:
                continue

    except Exception as e:
        logger.error(f"[Process for camera {camera_id}] An unhandled exception occurred: {e}", exc_info=True)
    finally:
        if gige_node:
            gige_node.stop_streaming()
            if streaming_thread:
                 streaming_thread.join()
            gige_node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        logger.info(f"[Process for camera {camera_id}] Terminated.")
