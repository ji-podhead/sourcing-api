import gi
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis

try:
    Aravis.update_device_list()
    device_id = Aravis.get_device_id(0)
    cam = Aravis.Camera.new(device_id)
    device = cam.props.device
    for attr in dir(device):
        print(attr)
except Exception as e:
    print(f"Error: {e}")
