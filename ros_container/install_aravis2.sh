#!/bin/bash
set -e
# --- SCHRITT 1: Systemabhängigkeiten installieren ---
echo ">>> [1/5] Systemabhängigkeiten werden installiert..."
# Apt-Quellen aktualisieren und 'universe' hinzufügen (wichtig für libaravis)
apt-get update
apt-get install -y software-properties-common git python3-rosdep
add-apt-repository universe
apt-get update
# Essenzielle Bibliothek für die Kamera installieren
apt-get install -y libaravis-dev meson || echo "Warnung: libaravis-dev konnte nicht installiert werden, fahre fort..."

# --- SCHRITT 2: ROS 2 Umgebung und rosdep vorbereiten ---
echo ">>> [2/5] ROS 2 Umgebung wird vorbereitet..."
source /opt/ros/humble/setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
rosdep update

# --- SCHRITT 3: ROS 2 Workspace erstellen und Code klonen ---
echo ">>> [3/5] ROS 2 Workspace wird erstellt..."
ROS_WORKSPACE=/root/ros2_ws
mkdir -p ${ROS_WORKSPACE}/src
cd ${ROS_WORKSPACE}/src
git clone https://github.com/FraunhoferIOSB/camera_aravis2.git

# --- SCHRITT 4: ROS 2 Abhängigkeiten installieren ---
echo ">>> [4/5] ROS 2 Abhängigkeiten werden mit rosdep installiert..."
cd ${ROS_WORKSPACE}
rosdep install --from-paths src --ignore-src -r -y

# --- SCHRITT 5: Workspace bauen ---
echo ">>> [5/5] Workspace wird mit colcon gebaut..."
source /opt/ros/humble/setup.bash
# Explicitly install a compatible version of setuptools
pip install --no-cache-dir setuptools==58.2.0

# Überprüfen ob die Datei existiert, bevor sie modifiziert wird
SETUP_FILE="/root/ros2_ws/src/camera_aravis2/camera_aravis2_msgs/setup.py"
if [ -f "$SETUP_FILE" ]; then
    # Apply the fix for the canonicalize_version error
    sed -i 's/strip_trailing_zero=False,//' "$SETUP_FILE"
else
    echo "Warnung: $SETUP_FILE wurde nicht gefunden. Dies ist normal für einige Versionen des Repositories."
fi

cd ${ROS_WORKSPACE}
colcon build --symlink-install
echo ">>> Installation abgeschlossen!"
echo ">>> Fügen Sie 'source ${ROS_WORKSPACE}/install/setup.bash' zu Ihrem Startbefehl hinzu."
