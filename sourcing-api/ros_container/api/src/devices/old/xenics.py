import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROS2Image
import gi
import sys

# Erforderliche GObject-Bibliotheken importieren
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis, GLib # GLib ist entscheidend!
TARGET_PIXEL_FORMAT = "Mono16" # Loaded from config/xenics.yml

class XenicsCameraNode(Node):
    def __init__(self):
        super().__init__('xenics_camera')
        
        # ROS 2 Publisher für das rohe Bild erstellen
        self.image_pub = self.create_publisher(ROS2Image, 'xenics_image_raw', 10)
        
        # Holen Sie sich den Standard-Hauptkontext von GLib.
        # Dies ist die Ereignisschleife, die wir manuell anstoßen müssen.
        self.glib_context = GLib.MainContext.default()
        
        # Initialisierungsvariablen
        self.camera = None
        self.stream = None
        
        try:
            # 1. Kamera initialisieren (Logik aus Ihrem ersten Code)
            self.get_logger().info("Suche nach Aravis-Kamera...")
            self.camera = Aravis.Camera.new("Xenics-Ceres-640-GigE-23255") # None sucht die erste verfügbare Kamera
            if self.camera is None:
                self.get_logger().error("Keine Aravis-Kamera gefunden!")
                raise Exception("Kamera nicht gefunden")
            self.get_logger().info(f"Kamera gefunden: {self.camera.get_model_name()}")

            # WICHTIG: Setzen Sie die Kamera in den kontinuierlichen Modus
            # Viele Kameras starten im "Trigger"-Modus und warten auf ein externes Signal.
            # WICHTIG: Setzen Sie die Kamera in den kontinuierlichen Modus
            # Viele Kameras starten im "Trigger"-Modus und warten auf ein externes Signal.
            # WICHTIG: Setzen Sie die Kamera in den kontinuierlichen Modus
            # Viele Kameras starten im "Trigger"-Modus und warten auf ein externes Signal.
            # The error message "'Camera' object has no attribute 'set_feature'" indicates
            # that the set_feature method is not available in this Aravis version.
            # We will skip setting the acquisition mode and rely on the camera's default.
            # If the default mode is not suitable, further investigation into the Aravis API
            # for this specific version will be needed.
            self.get_logger().warn("Skipping setting camera acquisition mode due to missing 'set_feature' attribute.")

            # Set camera features using available methods from the test script.
            try:
                # Set TriggerMode to FreeRunning
                self.camera.set_string("TriggerMode", "FreeRunning")
                self.get_logger().info("TriggerMode successfully set to 'FreeRunning'.")

                # Set PixelFormat to Mono16
                self.camera.set_pixel_format_from_string(TARGET_PIXEL_FORMAT) # Assuming TARGET_PIXEL_FORMAT is defined as "Mono16"
                self.get_logger().info(f"PixelFormat successfully set to '{TARGET_PIXEL_FORMAT}'.")

                # Get Width and Height using get_region()
                # The test script uses this to get width/height, so it's likely available.
                # We'll use these values for the ROS message.
                _, _, self.width, self.height = self.camera.get_region()
                self.get_logger().info(f"Camera resolution set to: {self.width}x{self.height}")

            except Exception as e:
                self.get_logger().warn(f"Could not set camera features: {e}")

            self.payload = self.camera.get_payload()
            # We already set the pixel format, so we can use that directly or re-read it.
            # Let's re-read it to be safe, or use the TARGET_PIXEL_FORMAT if we are confident.
            # For now, let's use the TARGET_PIXEL_FORMAT as we explicitly set it.
            self.pixel_format_str = TARGET_PIXEL_FORMAT # Or self.camera.get_pixel_format_as_string()
            self.get_logger().info(f"Payload: {self.payload} Bytes, Pixelformat: {self.pixel_format_str}")
            self.get_logger().info(f"Payload: {self.payload} Bytes, Pixelformat: {self.pixel_format_str}")

            # 2. Stream erstellen und Buffer vorbereiten
            self.stream = self.camera.create_stream(None, None)
            if self.stream is None:
                self.get_logger().error("Stream konnte nicht erstellt werden!")
                raise Exception("Stream-Fehler")

            for i in range(10): # 10 Buffer sind ein guter Startwert
                self.stream.push_buffer(Aravis.Buffer.new_allocate(self.payload))
            
            # 3. Den Callback für neue Bilder registrieren
            self.stream.connect('new-buffer', self.new_buffer_callback)
            
            # 4. Akquisition starten
            self.camera.start_acquisition()
            self.get_logger().info('Kamera-Akquisition gestartet.')

            # 5. DIE ENTSCHEIDENDE VERBINDUNG: ROS-Timer, der die GLib-Schleife antreibt
            # Ein 1ms-Timer sorgt für eine sehr geringe Latenz.
            self.glib_timer = self.create_timer(0.001, self.glib_main_loop_iteration)
            
            self.get_logger().info('Xenics Kamera-Node ist betriebsbereit.')

        except Exception as e:
            self.get_logger().fatal(f'Fehler bei der Kamera-Initialisierung: {str(e)}')
            rclpy.shutdown() # Beendet das Programm sauber, wenn die Kamera nicht funktioniert

    def glib_main_loop_iteration(self):
        """Diese Funktion wird vom ROS-Timer aufgerufen und verarbeitet anstehende GObject-Ereignisse."""
        # may_block=False ist wichtig, damit es nicht blockiert und den ROS-Node aufhält.
        self.glib_context.iteration(may_block=False)
        

    def destroy_node(self):
        # Sauberes Herunterfahren der Kamera
        if hasattr(self, 'camera') and self.camera is not None:
            self.get_logger().info('Stoppe Kamera-Akquisition.')
            self.camera.stop_acquisition()
        super().destroy_node()

    def new_buffer_callback(self, stream):
        """
        Dieser Callback wird von der GLib-Schleife aufgerufen, wenn ein neuer Frame verfügbar ist.
        Hier findet die eigentliche Arbeit statt.
        """
        # self.get_logger().debug('Neuer Buffer empfangen.') # Nützlich für Debugging
        buffer = stream.pop_buffer()
        if buffer is None:
            self.get_logger().warn("Received None buffer.")
            return

        try:
            # Check if the buffer contains data
            buffer_data = buffer.get_data()
            if not buffer_data:
                self.get_logger().warn("Buffer data is empty. Camera might not be acquiring frames correctly.")
                stream.push_buffer(buffer) # Still push the buffer back
                return

            # Eine leere ROS2 Image-Nachricht erstellen
            ros2_msg = ROS2Image()

            # Header füllen
            ros2_msg.header.stamp = self.get_clock().now().to_msg()
            ros2_msg.header.frame_id = "camera_link" 

            # Bild-Metadaten direkt von der Kamera holen und setzen
            ros2_msg.height = self.camera.get_integer_feature('Height')
            ros2_msg.width = self.camera.get_integer_feature('Width')
            
            # Encoding und Schrittweite (step) basierend auf dem Pixelformat bestimmen
            if self.pixel_format_str == "Mono16":
                ros2_msg.encoding = "mono16"
                bytes_per_pixel = 2
            elif self.pixel_format_str == "Mono8":
                ros2_msg.encoding = "mono8"
                bytes_per_pixel = 1
            else:
                self.get_logger().warn_once(f"Unbekanntes Pixelformat: {self.pixel_format_str}. Fallback auf 'mono8'.")
                ros2_msg.encoding = "mono8"
                bytes_per_pixel = 1
            
            ros2_msg.step = ros2_msg.width * bytes_per_pixel
            ros2_msg.is_bigendian = (sys.byteorder == 'big')

            # Die Rohdaten direkt aus dem Buffer in die Nachricht kopieren
            buffer_data = buffer.get_data()
            ros2_msg.data = buffer_data

            # Die vollständige ROS 2 Nachricht veröffentlichen
            self.image_pub.publish(ros2_msg)
            
        except Exception as e:
            self.get_logger().error(f'Fehler beim Verarbeiten des Bildes: {str(e)}')
        finally:
            # WICHTIG: Den Buffer wieder in den Stream pushen, damit er wiederverwendet werden kann
            stream.push_buffer(buffer)

def main(args=None):
    rclpy.init(args=args)
    
    xenics_node = XenicsCameraNode()
    
    # Prüfen, ob die Initialisierung rclpy.shutdown() aufgerufen hat
    if rclpy.ok():
        try:
            # spin() hält den Knoten am Laufen und lässt die Timer laufen
            rclpy.spin(xenics_node)
        except KeyboardInterrupt:
            pass
        finally:
            xenics_node.get_logger().info('Beende Xenics Kamera-Node...')
            # Aufräumen beim Beenden
            xenics_node.destroy_node()
            rclpy.shutdown()

    else:
        xenics_node.get_logger().fatal('Knoten konnte nicht gestartet werden, rclpy ist nicht bereit.')
        xenics_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
