# gige_camera_node.py
import roslibpy
import logging
import threading
import base64
import time
import sys
import signal
import numpy as np
import os
from typing import Optional

import gi
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis, GLib

# --- Konfiguration ---
RUNDIR = "/tmp/gige_pool"
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - [%(name)s] - %(message)s')

class CameraWorker:
    def __init__(self, worker_id: str, ros_host: str = 'localhost', ros_port: int = 9090):
        self.worker_id = worker_id
        self.logger = logging.getLogger(worker_id)
        self.ros_host = ros_host
        self.ros_port = ros_port

        self.ros_client: Optional[roslibpy.Ros] = None
        self.camera: Optional[Aravis.Camera] = None
        self.stream: Optional[Aravis.Stream] = None
        
        # --- NEU: Dedizierte Publisher für Bild und Status ---
        self.image_pub: Optional[roslibpy.Topic] = None
        self.status_publisher: Optional[roslibpy.Topic] = None
        
        self.acquisition_thread: Optional[threading.Thread] = None
        self.shutdown_event = threading.Event()
        self.active_camera_identifier: Optional[str] = None

    # --- NEU: Eine zentrale Funktion zum Senden von Statusmeldungen ---
    def _publish_status(self, message: str):
        """Protokolliert eine Nachricht und sendet sie an das ROS-Status-Topic."""
        timestamp = time.strftime('%H:%M:%S')
        full_message = f"[{timestamp}] {message}"
        self.logger.info(f"STATUS: {full_message}")
        if self.status_publisher and self.ros_client and self.ros_client.is_connected:
            msg = roslibpy.Message({'data': full_message})
            self.status_publisher.publish(msg)

    def start_camera_stream(self, camera_identifier: str):
        if self.acquisition_thread and self.acquisition_thread.is_alive():
            self._publish_status(f"WARNUNG: Bereits aktiv. Stoppe alten Stream zuerst.")
            self.stop_camera_stream()

        self._publish_status(f"START-Befehl für '{camera_identifier}' erhalten.")
        self.active_camera_identifier = camera_identifier
        self.shutdown_event.clear()

        try:
            self.camera = Aravis.Camera.new(self.active_camera_identifier)
            if self.camera is None:
                raise Exception(f"Kamera '{self.active_camera_identifier}' nicht gefunden.")
            
            [x, y, self.width, self.height] = self.camera.get_region()
            self._publish_status(f"Kamera gefunden: {self.width}x{self.height}")

            self.stream = self.camera.create_stream(None, None)
            payload = self.camera.get_payload()
            for _ in range(10):
                self.stream.push_buffer(Aravis.Buffer.new_allocate(payload))

            topic_name = f"/gige_camera/{self.worker_id}/image_raw"
            self.image_pub = roslibpy.Topic(self.ros_client, topic_name, 'sensor_msgs/msg/Image')
            self._publish_status(f"Advertising Bild-Topic: {topic_name}")

            self.acquisition_thread = threading.Thread(target=self._acquisition_loop, name=f"{self.worker_id}-Acq")
            self.acquisition_thread.start()
        except Exception as e:
            self._publish_status(f"FEHLER beim Starten der Kamera: {e}")
            self.logger.error(f"Failed to start camera: {e}", exc_info=True)
            self.stop_camera_stream()

    def stop_camera_stream(self):
        self._publish_status("STOP-Befehl erhalten.")
        self.shutdown_event.set()
        if self.acquisition_thread:
            self.acquisition_thread.join(timeout=3.0)
        
        if self.camera and self.camera.is_acquiring():
            try: self.camera.stop_acquisition()
            except GLib.Error: pass

        if self.image_pub:
            self.image_pub.unadvertise()
            self._publish_status(f"Bild-Topic unadvertised.")

        self.camera = None
        self.stream = None
        self.image_pub = None
        self.active_camera_identifier = None
        self.acquisition_thread = None
        self._publish_status("Worker ist jetzt IDLE.")

    def _acquisition_loop(self):
        self.camera.start_acquisition()
        self._publish_status("Akquisition gestartet. Sende Bilder...")
        while not self.shutdown_event.is_set():
            buffer = self.stream.pop_buffer()
            if buffer:
                try:
                    if buffer.get_status() == Aravis.BufferStatus.SUCCESS:
                        image_data = buffer.get_data()
                        ros_msg = { 'header': {'stamp': {'sec': int(time.time()), 'nanosec': 0}, 'frame_id': self.worker_id},
                                    'height': self.height, 'width': self.width, 'encoding': "BayerRG8", 'is_bigendian': False,
                                    'step': self.width, 'data': base64.b64encode(image_data).decode('utf-8') }
                        if self.image_pub and self.ros_client.is_connected:
                            self.image_pub.publish(roslibpy.Message(ros_msg))
                finally:
                    self.stream.push_buffer(buffer)
        self._publish_status("Akquisitions-Schleife beendet.")

    def _on_ros_connect(self):
        """Wird aufgerufen, wenn die ROS-Verbindung steht."""
        self.logger.info("Connection to ROS ready.")
        # Status-Publisher erstellen, sobald die Verbindung steht
        status_topic_name = f"/gige_camera/{self.worker_id}/status"
        self.status_publisher = roslibpy.Topic(self.ros_client, status_topic_name, 'std_msgs/String')
        # Warte einen Moment, um sicherzustellen, dass das Topic advertised wurde
        time.sleep(0.5)
        self._publish_status("Worker ROS-Verbindung bereit. Status: IDLE.")
        
    def run_ros_loop(self):
        """Startet den ROS-Client und blockiert für immer."""
        self.ros_client = roslibpy.Ros(host=self.ros_host, port=self.ros_port)
        # --- NEU: Den on_ready Callback registrieren ---
        self.ros_client.on_ready(self._on_ros_connect)
        try:
            self.logger.info(f"Starting ROS main loop...")
            self.ros_client.run_forever()
        finally:
            self.logger.info("ROS loop terminated.")
            if self.ros_client.is_connected:
                self.ros_client.terminate()

def main(worker_id):
    os.makedirs(RUNDIR, exist_ok=True)
    pid_file = os.path.join(RUNDIR, f"{worker_id}.pid")
    pipe_file = os.path.join(RUNDIR, f"{worker_id}.pipe")

    with open(pid_file, 'w') as f: f.write(str(os.getpid()))
    if os.path.exists(pipe_file): os.remove(pipe_file)
    os.mkfifo(pipe_file, 0o666)

    worker = CameraWorker(worker_id=worker_id)
    logger = worker.logger

    def command_listener():
        logger.info(f"Command listener started. Waiting for commands on {pipe_file}")
        while True:
            try:
                with open(pipe_file, 'r') as fifo:
                    command_line = fifo.readline().strip()
                if not command_line:
                    time.sleep(0.1); continue
                
                parts = command_line.split(' ', 1)
                command = parts[0].upper()
                if command == "START":
                    worker.start_camera_stream(parts[1]) if len(parts) > 1 else logger.error("START needs camera_id")
                elif command == "STOP":
                    worker.stop_camera_stream()
                elif command == "EXIT":
                    os.kill(os.getpid(), signal.SIGTERM); break
            except Exception as e:
                logger.error(f"Error in command listener: {e}", exc_info=True); time.sleep(1)

    def cleanup(signum, frame):
        logger.info(f"Signal {signal.Signals(signum).name} received. Cleaning up...")
        worker._publish_status("Worker wird heruntergefahren...")
        worker.stop_camera_stream()
        if os.path.exists(pid_file): os.remove(pid_file)
        if os.path.exists(pipe_file): os.remove(pipe_file)
        if worker.ros_client: worker.ros_client.terminate()
        logger.info("Cleanup complete. Exiting.")

    signal.signal(signal.SIGTERM, cleanup)
    signal.signal(signal.SIGINT, cleanup)

    listener_thread = threading.Thread(target=command_listener, name=f"{worker_id}-Listener")
    listener_thread.daemon = True
    listener_thread.start()

    worker.run_ros_loop()
    logger.info(f"Main thread exiting.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python gige_camera_node.py <worker_id>"); sys.exit(1)
    main(sys.argv[1])