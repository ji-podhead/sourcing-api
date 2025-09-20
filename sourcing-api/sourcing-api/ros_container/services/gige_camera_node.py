# gige_camera_node.py
import roslibpy
import logging
import threading
import base64
import time
import sys
import signal
import numpy as np
from typing import Optional
import os

import gi
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis, GLib

# --- Konfiguration ---
RUNDIR = "/tmp/gige_pool"  # Verzeichnis für PID-Dateien und Pipes
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - [%(name)s] - %(message)s')

# Die CameraWorker-Klasse bleibt größtenteils gleich, aber ohne Flask.
# Die Kommunikation läuft jetzt über die main-Schleife und eine Pipe.
class CameraWorker:
    def __init__(self, worker_id: str, ros_host: str = 'localhost', ros_port: int = 9090):
        self.worker_id = worker_id
        self.logger = logging.getLogger(worker_id)
        self.ros_host = ros_host
        self.ros_port = ros_port

        self.ros_client: Optional[roslibpy.Ros] = None
        self.camera: Optional[Aravis.Camera] = None
        self.stream: Optional[Aravis.Stream] = None
        self.image_pub: Optional[roslibpy.Topic] = None
        self.acquisition_thread: Optional[threading.Thread] = None
        self.shutdown_event = threading.Event()
        self.active_camera_identifier: Optional[str] = None

    def start_camera_stream(self, camera_identifier: str):
        if self.acquisition_thread and self.acquisition_thread.is_alive():
            self.stop_camera_stream()
            self.logger.warning(f"Worker is already running a stream for '{self.active_camera_identifier}'. Stopping it first.")

        self.logger.info(f"Command received: START stream for '{camera_identifier}'")
        self.active_camera_identifier = camera_identifier
        self.shutdown_event.clear()

        try:
            self.camera = Aravis.Camera.new(self.active_camera_identifier)
            if self.camera is None:
                raise Exception(f"Camera '{self.active_camera_identifier}' not found.")
            
            [x, y, self.width, self.height] = self.camera.get_region()
            self.pixel_format_str = self.camera.get_pixel_format_as_string()
            self.logger.info(f"Camera found: {self.width}x{self.height} @ {self.pixel_format_str}")

            self.stream = self.camera.create_stream(None, None)
            payload = self.camera.get_payload()
            for _ in range(100):
                self.stream.push_buffer(Aravis.Buffer.new_allocate(payload))

            topic_name = f"/gige_camera/{self.worker_id}/image_raw"
            self.image_pub = roslibpy.Topic(self.ros_client, topic_name, 'sensor_msgs/msg/Image')
            self.logger.info(f"Advertising ROS topic: {topic_name}")
            self.ros_client.run()

            self.acquisition_thread = threading.Thread(target=self._acquisition_loop, name=f"{self.worker_id}-Acq")
            self.acquisition_thread.start()
        except Exception as e:
            self.logger.error(f"Failed to start camera: {e}", exc_info=True)
            self.stop_camera_stream() # Aufräumen bei Fehler

    def stop_camera_stream(self):
        self.logger.info("Command received: STOP stream")
        self.shutdown_event.set()
        if self.acquisition_thread:
            self.acquisition_thread.join(timeout=3.0)
        
        if self.camera and self.camera.is_acquiring():
            try: self.camera.stop_acquisition()
            except GLib.Error: pass

        if self.image_pub:
            self.image_pub.unadvertise()
            self.logger.info(f"Unadvertised ROS topic.")

        self.camera = None
        self.stream = None
        self.image_pub = None
        self.active_camera_identifier = None
        self.acquisition_thread = None
        self.logger.info("Camera stream and resources released. Worker is idle.")

    def _acquisition_loop(self):
        self.camera.start_acquisition()
        self.logger.info("Acquisition loop started.")
        while not self.shutdown_event.is_set():
            buffer = self.stream.pop_buffer()
            if buffer:
                try:
                    if buffer.get_status() == Aravis.BufferStatus.SUCCESS:
                        # Publish image data
                        image_data = buffer.get_data()
                        current_time = time.time()
                        seconds, nanoseconds = divmod(current_time, 1)
                        encoding = "mono8"  # Default value
                        if self.pixel_format_str == "aravis- BayerGR8":
                            encoding = "bayer_grbg8"
                        elif self.pixel_format_str == "aravis-RGB8":
                            encoding = "rgb8"
                        ros_msg = { 'header': {'stamp': {'sec': int(seconds), 'nanosec': int(nanoseconds * 1e9)}, 'frame_id': self.worker_id},
                                    'height': self.height, 'width': self.width, 'encoding': encoding, 'is_bigendian': False,
                                    'step': self.width, 'data': base64.b64encode(image_data).decode('utf-8') }
                        if self.image_pub and self.ros_client.is_connected:
                            try:
                                # Frame verarbeiten und publishen
                                image_data = buffer.get_data()
                                frame_8bit = np.ndarray(buffer=image_data, dtype=np.uint8, shape=(self.height, self.width)).copy()

                                # ROS Image erstellen und publishen
                                msg = {
                                    'header': {'stamp': {'sec': int(time.time()), 'nanosec': 0}, 'frame_id': self.worker_id},
                                    'height': self.height,
                                    'width': self.width,
                                    'encoding': 'mono8',
                                    'is_bigendian': False,
                                    'step': self.width,
                                    'data': base64.b64encode(image_data).decode('utf-8')
                                }
                                self.image_pub.publish(roslibpy.Message(msg))
                                self.logger.info("✓ Bild veröffentlicht")

                            except Exception as e:
                                self.logger.error(f"Error publishing image: {e}", exc_info=True)
                        else:
                            self.logger.warning("ROS client not connected or image_pub not initialized.")
                finally:
                    self.stream.push_buffer(buffer)
        self.logger.info("Acquisition loop finished.")

    def run_ros_loop(self):
        """Startet den ROS-Client und blockiert für immer."""
        self.ros_client = roslibpy.Ros(host=self.ros_host, port=self.ros_port)
        try:
            self.logger.info(f"Starting ROS main loop for worker {self.worker_id}...")
            topic_name = f"/gige_camera/{self.worker_id}/status"
            status_pub = roslibpy.Topic(self.ros_client, topic_name, 'std_msgs/msg/String')
            status_pub.advertise()
            message = roslibpy.Message({'data': "online"})
            status_pub.publish(message)
            print(f"Published status 'online' to topic '{topic_name}'")
            # status_pub.unadvertise()
            self.ros_client.run_forever()
        finally:
            self.logger.info("ROS loop terminated.")
            if self.ros_client.is_connected:
                self.ros_client.terminate()

def publish_worker_status(ros_client, worker_id, status):
    """Veröffentlicht den Status des Workers."""
    topic_name = f"/gige_camera/{worker_id}/status"
    status_pub = roslibpy.Topic(ros_client, topic_name, 'std_msgs/msg/String')
    status_pub.advertise()
    message = roslibpy.Message({'data': status})
    status_pub.publish(message)
    print(f"Published status '{status}' to topic '{topic_name}'")
    status_pub.unadvertise()


def main(worker_id):
    # --- Setup ---
    os.makedirs(RUNDIR, exist_ok=True)
    pid_file = os.path.join(RUNDIR, f"{worker_id}.pid")
    pipe_file = os.path.join(RUNDIR, f"{worker_id}.pipe")

    # PID-Datei schreiben
    with open(pid_file, 'w') as f:
        f.write(str(os.getpid()))

    # Named Pipe erstellen
    if os.path.exists(pipe_file):
        os.remove(pipe_file)
    os.mkfifo(pipe_file, 0o666) # Berechtigungen für alle Nutzer setzen

    worker = CameraWorker(worker_id=worker_id)
    logger = worker.logger

    def command_listener():
        """Diese Funktion läuft im Hintergrund und lauscht auf der Pipe."""
        logger.info(f"Command listener started. Waiting for commands on {pipe_file}")
        while True:
            try:
                with open(pipe_file, 'r') as fifo:
                    command_line = fifo.readline().strip()
                
                if not command_line:
                    time.sleep(0.1)
                    continue

                logger.info(f"Received raw command: '{command_line}'")
                parts = command_line.split(' ', 1)
                command = parts[0].upper()
                
                if command == "START":
                    logger.info("START command received.")
                    print(parts)
                    print("starting")
                    if worker.ros_client and worker.ros_client.is_connected:
                        logger.info("ROS client is connected.")
                        if len(parts) > 1:
                            worker.start_camera_stream(parts[1])
                        else:
                            logger.error("START command requires a camera_identifier.")
                    else:
                        logger.warning("ROS client not connected yet. Command execution is delayed.")
                elif command == "STOP":
                    worker.stop_camera_stream()
                elif command == "EXIT":
                    # Beende den Hauptprozess sauber
                    os.kill(os.getpid(), signal.SIGTERM)
                    break # Beende den Listener-Thread
                else:
                    logger.warning(f"Unknown command: {command}")
            except Exception as e:
                logger.error(f"Error in command listener: {e}", exc_info=True)
                time.sleep(1)


    def cleanup(signum, frame):
        logger.info(f"Signal {signal.Signals(signum).name} received. Cleaning up...")
        worker.stop_camera_stream()
        if os.path.exists(pid_file): os.remove(pid_file)
        if os.path.exists(pipe_file): os.remove(pipe_file)
        logger.info("Cleanup complete. Exiting.")
        # Beende den Haupt-ROS-Loop
        if worker.ros_client:
            worker.ros_client.terminate()
        # sys.exit(0) ist hier nicht nötig, da terminate() den Loop beendet

    # --- KORREKTUR: ROLLENTAUSCH ---

    # 1. Signal-Handler im Haupt-Thread registrieren (jetzt korrekt)
    signal.signal(signal.SIGTERM, cleanup)
    signal.signal(signal.SIGINT, cleanup)

    # 2. Den Pipe-Listener im Hintergrund-Thread starten
    listener_thread = threading.Thread(target=command_listener, name=f"{worker_id}-Listener")
    listener_thread.daemon = True
    listener_thread.start()

    # 3. Den ROS-Loop im Haupt-Thread ausführen (jetzt korrekt)
    worker.run_ros_loop()
    
    # Dieser Punkt wird nur erreicht, wenn run_forever() beendet wird (z.B. durch cleanup)
    logger.info(f"[{worker_id}] Main thread exiting.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python gige_camera_node.py <worker_id>")
        sys.exit(1)
    main(sys.argv[1])
