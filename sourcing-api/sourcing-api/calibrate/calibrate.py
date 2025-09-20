import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import roslibpy
import json
import cv2
import numpy as np
import os
import math
import gi
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis

# --- Konfiguration ---
CHESSBOARD_SIZE = (9, 6)  # Anzahl der INNEREN Ecken (Spalten, Zeilen)
SQUARE_SIZE_MM = 25.0 # Die tatsächliche Größe eines Quadrats in mm
MIN_IMAGES_FOR_CALIBRATION = 15
ROS_TOPIC = '/camera/image_raw'
LOG_TOPIC = '/camera/logs'

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        
        # ROS2 Node initialisieren
        self.get_logger().info('Initialisiere Kamera-Kalibrierungs-Node...')
        
        # Publisher für Bilder
        self.image_publisher = self.create_publisher(Image, ROS_TOPIC, 10)
        self.get_logger().info('✓ Bild-Publisher erstellt')
        
        # Publisher für Log-Nachrichten
        self.log_publisher = self.create_publisher(String, LOG_TOPIC, 10)
        self.get_logger().info('✓ Log-Publisher erstellt')
        
        # Timer für periodische Log-Nachrichten
        self.timer = self.create_timer(1.0, self.log_callback)
        
        # ROS Client für die Kamera-Kommunikation
        self.ros_client = None
        self.camera = None
        self.stream = None
        
    def log_callback(self):
        """Sendet periodische Log-Nachrichten"""
        log_msg = String()
        log_msg.data = f'Kamera-Status: {self.get_clock().now().to_msg()}'
        self.log_publisher.publish(log_msg)
        self.get_logger().debug(f'Log-Nachricht veröffentlicht: {log_msg.data}')
    
    def publish_log(self, message):
        """Hilfsfunktion zum Veröffentlichen von Log-Nachrichten"""
        log_msg = String()
        log_msg.data = message
        self.log_publisher.publish(log_msg)
        self.get_logger().info(f'Log veröffentlicht: {message}')
    
    def normalize_frame_16bit(self, frame_16bit):
        """Normalisiert einen 16-Bit-Frame in einen 8-Bit-Frame"""
        min_val = np.min(frame_16bit)
        max_val = np.max(frame_16bit)
        if max_val == min_val:
            return np.zeros(frame_16bit.shape, dtype=np.uint8)
        normalized = (frame_16bit - min_val) / (max_val - min_val)
        return (normalized * 255.0).astype(np.uint8)
    
    def calibrate_camera(self, objpoints, imgpoints, frame_shape):
        """Führt die Kamerakalibrierung durch"""
        self.publish_log("Starte Kamerakalibrierung...")
        print("\nFühre Kalibrierung durch...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frame_shape, None, None)
        if not ret:
            print("Kalibrierung fehlgeschlagen!")
            self.publish_log("Kalibrierung fehlgeschlagen!")
            return None, None
        
        print("Kalibrierung erfolgreich!")
        self.publish_log("Kalibrierung erfolgreich!")
        print("\n--- Ergebnisse ---")
        print("Kameramatrix (K):")
        print(mtx)
        print("\nDistortionskoeffizienten (D):")
        print(dist)
        
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        
        print(f"\nDurchschnittlicher Reprojektionsfehler: {mean_error/len(objpoints):.4f} Pixel")
        print("Ein Wert < 1.0 ist gut.")
        self.publish_log(f"Reprojektionsfehler: {mean_error/len(objpoints):.4f} Pixel")
        return mtx, dist
    
    def calibrate_and_publish(self):
        """Hauptfunktion für Kalibrierung und Publishing"""
        self.publish_log("Starte Kalibrierungs- und Publishing-Prozess...")
        
        try:
            # ROS Client initialisieren
            self.ros_client = roslibpy.Ros(host='localhost', port=9090)
            self.ros_client.run()
            self.publish_log("✓ ROS Client verbunden")
            
            # Kamera initialisieren
            Aravis.update_device_list()
            if Aravis.get_n_devices() == 0:
                self.publish_log("✗ Keine Kamera gefunden!")
                return
            
            self.publish_log(f"✓ Gefundene Kamera: {Aravis.get_device_id(0)}")
            self.camera = Aravis.Camera.new(Aravis.get_device_id(0))
            
            # Stream starten
            self.stream = self.camera.create_stream()
            payload = self.camera.get_payload()
            [x, y, width, height] = self.camera.get_region()
            self.publish_log(f"✓ Auflösung: {width}x{height}")
            
            # Puffer vorbereiten
            for i in range(10):
                self.stream.push_buffer(Aravis.Buffer.new_allocate(payload))
            
            self.camera.start_acquisition()
            self.publish_log("✓ Kamerastream gestartet")
            
            # Arrays für Kalibrierung vorbereiten
            objpoints = []
            imgpoints = []
            
            while rclpy.ok():
                self.publish_log("Warte auf neuen Frame...")
                buffer = self.stream.pop_buffer()
                
                if buffer:
                    self.publish_log("✓ Buffer empfangen")
                    payload = buffer.get_data()
                    frame_8bit = np.ndarray(buffer=payload, dtype=np.uint8, shape=(height, width)).copy()
                    
                    # Frame verarbeiten und publishen
                    gray = cv2.cvtColor(frame_8bit, cv2.COLOR_BayerRG2GRAY)
                    gray_normalized = self.normalize_frame_16bit(gray)
                    
                    # ROS Image erstellen und publishen
                    msg = Image()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.height = height
                    msg.width = width
                    msg.encoding = 'mono8'
                    msg.step = width
                    msg.data = gray_normalized.tobytes()
                    
                    self.image_publisher.publish(msg)
                    self.publish_log("✓ Bild veröffentlicht")
                    
                    # Buffer zurückgeben
                    self.stream.push_buffer(buffer)
                    
                    # Warten auf nächsten Frame
                    rclpy.spin_once(self, timeout_sec=0.1)
                else:
                    self.publish_log("✗ Kein Buffer empfangen")
        
        except Exception as e:
            self.publish_log(f"✗ Fehler aufgetreten: {str(e)}")
            raise
        
        finally:
            if self.camera:
                self.camera.stop_acquisition()
                self.publish_log("✓ Kamera gestoppt")
            if self.ros_client:
                self.ros_client.terminate()
                self.publish_log("✓ ROS Client beendet")
    
    def destroy_node(self):
        """Aufräumen beim Beenden des Nodes"""
        self.get_logger().info('Bereinige Kamera-Kalibrierungs-Node...')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    camera_node = CameraCalibrationNode()
    try:
        camera_node.calibrate_and_publish()
    except KeyboardInterrupt:
        camera_node.get_logger().info('Benutzer hat das Programm beendet')
    except Exception as e:
        camera_node.get_logger().error(f'Fehler: {str(e)}')
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()