import roslibpy
import threading
import time
import asyncio
import logging
from typing import Dict, Any, Tuple

logger = logging.getLogger(__name__)

class GigECameraNode:
    def __init__(self, camera_id: str, ros_host: str = 'localhost', ros_port: int = 9090):
        self.camera_id = camera_id
        self.is_running = False
        self._thread = None
        
        # Jede Kamera-Instanz bekommt ihren eigenen ROS-Client
        self.ros_client = roslibpy.Ros(host=ros_host, port=ros_port)
        
        # Hier würden wir Publisher und Service-Clients initialisieren
        # Beispiel: Ein Service zum Setzen von Features
        # Wichtig: Der Service-Name muss den Namen der Kamera enthalten, um eindeutig zu sein!
        self.set_feature_service_name = f"/gige_camera_{self.camera_id.replace('.', '_')}/set_feature"
        self.set_feature_service = roslibpy.Service(self.ros_client, 
                                                     self.set_feature_service_name, 
                                                     'interfaces/srv/SetFeature') # Beispiel für einen Service-Typ

    def _run_ros_connection(self):
        """Diese Funktion läuft in einem separaten Thread."""
        try:
            logger.info(f"[{self.camera_id}] Connecting to rosbridge...")
            # run_forever() blockiert, bis terminate() aufgerufen wird.
            self.ros_client.run_forever() 
        except Exception as e:
            logger.error(f"[{self.camera_id}] ROS connection error: {e}")
        finally:
            logger.info(f"[{self.camera_id}] ROS connection thread terminated.")
            self.is_running = False

    def start(self):
        """Startet den ROS-Client in einem Hintergrund-Thread."""
        if self.is_running:
            logger.warning(f"[{self.camera_id}] Start called but already running.")
            return

        self.is_running = True
        # Wir starten die blockierende `run_forever` in einem Daemon-Thread.
        # So blockiert es nicht unsere FastAPI-App.
        self._thread = threading.Thread(target=self._run_ros_connection, daemon=True)
        self._thread.start()
        logger.info(f"[{self.camera_id}] ROS connection thread started.")
        # Kurze Pause, um der Verbindung Zeit zu geben
        time.sleep(1) 

    def shutdown(self):
        """Beendet die ROS-Verbindung und den Thread."""
        if not self.is_running:
            return
        
        logger.info(f"[{self.camera_id}] Shutting down ROS connection.")
        self.ros_client.terminate()
        self._thread.join(timeout=2) # Auf das Ende des Threads warten
        self.is_running = False

    async def set_feature(self, feature_name: str, value: Any) -> Tuple[bool, str]:
        """
        Ruft den ROS-Service asynchron auf, um ein Feature zu setzen.
        Das ist die Echtzeit-Steuerung!
        """
        if not self.ros_client.is_connected:
            return False, "Not connected to ROS"

        request = roslibpy.ServiceRequest({
            'feature_name': str(feature_name),
            'value': str(value) # Service-Definition entscheidet über den Typ
        })
        
        loop = asyncio.get_event_loop()
        
        try:
            logger.info(f"[{self.camera_id}] Calling service '{self.set_feature_service_name}'...")
            # Da service.call() blockierend ist, führen wir es in einem Executor aus,
            # um den FastAPI-Event-Loop nicht zu blockieren.
            response = await loop.run_in_executor(
                None,  # Standard-Executor
                lambda: self.set_feature_service.call(request, timeout=5)
            )
            logger.info(f"[{self.camera_id}] Service response: {response}")
            return response['success'], response.get('message', '')
        except Exception as e:
            logger.error(f"[{self.camera_id}] Error calling service: {e}")
            return False, str(e)

    async def get_features(self) -> Dict[str, Any]:
        # Ähnliche Logik wie bei set_feature, aber für einen Service, der Features abfragt
        # ... Implementierung hier ...
        logger.info(f"[{self.camera_id}] Getting features (simulated).")
        # Platzhalter-Implementierung
        return {"PixelFormat": "Mono8", "ExposureTime": 10000}
