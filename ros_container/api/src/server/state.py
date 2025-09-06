import subprocess
from typing import Dict, Optional
from devices.gig_e_driver import GigECameraNode

driver_processes: Dict[str, subprocess.Popen] = {}
gige_camera_nodes: Dict[str, GigECameraNode] = {}
recording_process: Optional[subprocess.Popen] = None
