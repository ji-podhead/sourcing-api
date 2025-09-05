
import React, { useState, useEffect, useCallback, useRef } from "react";
import yaml from 'js-yaml'; // Import js-yaml for YAML parsing/dumping
import Link from "next/link"; // Import Link for navigation
import dynamic from 'next/dynamic'; // Import dynamic for SSR control
import { useWindowSize } from '../handlers/WindowSizeContext'; // Import the custom hook
import { useSelector, useDispatch } from 'react-redux';
import { RootState } from '../redux/store'; // Ensure this is the correct import for your store type
import {
  setSelectedFeatureGroup,
  setActiveTab,
  setIsSettingsOpen,
  setEditingNotes,
  setPresets,
  setSelectedPresetForEditing,
  setPresetForm,
  setPresetFormError,
  setRefreshInterval,
  setRecordingStatus,
  setRecordings,
  setMessage,
  setShowMessage,
} from '../redux/dashboardSlice';
import { setSelectedCamera, setDynamicCameras, setOusterStatus } from '../redux/devicesSlice';
import { DeviceStatus, CameraFeatureGroup, CameraFeature, StaticDeviceInfo, Preset } from '../types';

const WebTerminal = dynamic(() => import("./WebTerminal"), { ssr: false }); // Dynamically import WebTerminal with SSR disabled
import Sidebar from "./Sidebar";
import FeaturePanel from "./FeaturePanel";
import PresetsPanel from "./PresetsPanel";
import Settings from "./Settings";


const ROS_API_URL = "http://localhost:8000";
const WEBSOCKET_URL = "ws://localhost:8000/logs";
const TERMINAL_URL = "ws://localhost:8000/terminal";

interface DeviceSelectorPanelProps {
    RootState: any; // Replace 'any' with the actual type of your RootState
    dynamicCameras: any[];
    selectedCamera: any;
    setSelectedCamera: (cam: any) => void;
    setSelectedFeatureGroup: (group: string | null) => void;
    newCameraId: string;
    setNewCameraId: (id: string) => void;
    newCameraProtocol: string;
    setNewCameraProtocol: (protocol: string) => void;
    handleCreateCamera: () => void;
    handleAction: (
        endpoint: string,
        successMessage: string,
        errorMessage: string,
        device: any
    ) => void;
    setDeviceConfigs: (configs: any) => void;
    setMessage: (msg: string) => void;
    setShowMessage: (show: boolean) => void;
    setOusterStatus: (status: string) => void;
    setDynamicCameras: (cams: any[]) => void;
    ROS_API_URL: string;
    actionsRef: React.MutableRefObject<{
        fetchCameras: () => void;
        fetchRecordings: () => void;
    }>;
}


const DeviceSelectorPanel: React.FC<DeviceSelectorPanelProps> = ({
    RootState,
}) => {
  const dynamicCameras = useSelector((state: RootState) => state.devices.dynamicCameras);
  const selectedCamera = useSelector((state: RootState) => state.devices.selectedCamera);
  const ousterStatus = useSelector((state: RootState) => state.devices.ousterStatus);
  const selectedFeatureGroup = useSelector((state: RootState) => state.dashboard.selectedFeatureGroup);
  const activeTab = useSelector((state: RootState) => state.dashboard.activeTab);
  const isSettingsOpen = useSelector((state: RootState) => state.dashboard.isSettingsOpen);
  const editingNotes = useSelector((state: RootState) => state.dashboard.editingNotes);
  const presets = useSelector((state: RootState) => state.dashboard.presets);
  const selectedPresetForEditing = useSelector((state: RootState) => state.dashboard.selectedPresetForEditing);
  const presetForm = useSelector((state: RootState) => state.dashboard.presetForm);
  const presetFormError = useSelector((state: RootState) => state.dashboard.presetFormError);
  const refreshInterval = useSelector((state: RootState) => state.dashboard.refreshInterval);
  const recordingStatus = useSelector((state: RootState) => state.dashboard.recordingStatus);
  const recordings = useSelector((state: RootState) => state.dashboard.recordings);
  const message = useSelector((state: RootState) => state.dashboard.message);
  const showMessage = useSelector((state: RootState) => state.dashboard.showMessage);
  const ws = useRef<WebSocket | null>(null); // Ref for WebSocket instance
  const activeTabRef = useRef(activeTab);
  const selectedCameraRef = useRef(selectedCamera);

    // Handler for creating a new camera
    const handleCreateCamera = async () => {
        setMessage(""); // Clear previous messages
        if (!newCameraId.trim()) { // Add validation for camera ID
            setMessage("Camera ID (IP) cannot be empty.");
            setShowMessage(true);
            return;
        }

        // Add IP address validation for GigE protocol
        if (newCameraProtocol === 'gigE') {
            const ipRegex = /^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
            if (!ipRegex.test(newCameraId)) {
                setMessage("Invalid IP address format for GigE camera.");
                setShowMessage(true);
                return;
            }
        }

        try {
            const response = await fetch(`${ROS_API_URL}/create_camera`, { // Assuming /cameras is the new endpoint
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ // Corrected JSON.JSON.stringify to JSON.stringify
                    id: newCameraId,
                    protocol: newCameraProtocol,
                }),
            });
            const data = await response.json();
            if (response.ok) {
                setMessage(`Camera ${newCameraId} created successfully!`);
                setShowMessage(true);
                // Refresh the list of cameras
                actionsRef.current.fetchCameras();
                setNewCameraId(''); // Clear input fields
                setNewCameraProtocol('gigE'); // Reset protocol
            } else {
                setMessage(`Error creating camera: ${data.detail || response.statusText}`);
                setShowMessage(true);
            }
        } catch (error: unknown) { // Added type annotation for error
            setMessage(`Error creating camera: ${(error as Error).message}`);
            setShowMessage(true);
        }
    };

    // Generalize handleAction for dynamic cameras
    const handleAction = async (
        endpoint: string,
        successMessage: string,
        errorMessage: string,
        device: DeviceStatus // Pass the device object
    ) => {
        setMessage(""); // Clear previous messages
        try {
            let requestBody: any = {};
            let headers: HeadersInit = { "Content-Type": "application/json" };

            if (device.type === "gigE") {
                requestBody = {
                    protocol: device.type,
                    camera_id: device.id,
                };
            }
            // For other device types, the backend expects no body or different parameters.
            // The existing subprocess logic in main.py for "ouster", "xenics" doesn't expect a body for start/stop.
            // So, we only send a body if it's a gigE camera.

            const response = await fetch(`${ROS_API_URL}${endpoint}`, {
                method: "POST",
                headers: headers,
                body: device.type === "gigE" ? JSON.stringify(requestBody) : undefined, // Only send body for gigE
            });
            const data = await response.json();
            if (response.ok) {
                setMessage(successMessage + (data.message ? ` ${data.message}` : ""));

                // Update status based on device type
                if (device.type === "gigE") { // Use device.type for dynamic cameras
                    // Update status for dynamic camera in the dynamicCameras state
                    setDynamicCameras(prevCameras =>
                        prevCameras.map(cam =>
                            cam.id === device.id // Use device.id for dynamic cameras
                                ? { ...cam, status: data.status === "success" ? "running" : "stopped" }
                                : cam
                        )
                    );
                    if (selectedCamera && selectedCamera.id === device.id) {
                        setSelectedCamera(prev => prev ? { ...prev, status: data.status === "success" ? "running" : "stopped" } : null);
                    }
                } else if (device.name === "Ouster Lidar") {
                    setOusterStatus(data.status === "success" ? "running" : "stopped");
                }
                actionsRef.current.fetchRecordings();
            } else {
                setMessage(`${errorMessage}: ${data.detail || response.statusText}`);
            }
        } catch (error: unknown) { // Added type annotation for error
            setMessage(`${errorMessage}: ${(error as Error).message}`);
        }
    };

    return (
        <div className="flex items-center space-x-4">
            {/* New Camera Creation Section */}
            <div style={{ display: 'flex', gap: '10px', alignItems: 'center', backgroundColor: '#f9f9f9', padding: '10px', borderRadius: '8px' }}>
                <h3 className="text-lg font-semibold">Add a Device</h3>
                <label htmlFor="cameraId" className="sr-only">Device ID (IP):</label>
                <input
                    type="text"
                    id="cameraId"
                    placeholder="e.g., 192.168.1.100"
                    style={{ padding: '8px', border: '1px solid #ccc', borderRadius: '4px' }}
                    value={newCameraId}
                    onChange={e => setNewCameraId(e.target.value)}
                />
                <label htmlFor="protocol" className="sr-only">Protocol:</label>
                <select
                    id="protocol"
                    style={{ padding: '8px', border: '1px solid #ccc', borderRadius: '4px' }}
                    value={newCameraProtocol}
                    onChange={e => setNewCameraProtocol(e.target.value)}
                >
                    <option value="gigE">GigE</option>
                    <option value="gmsl2">GMSL2</option>
                </select>
                <button
                    onClick={handleCreateCamera}
                    style={{ padding: '10px 15px', backgroundColor: '#007bff', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
                >
                    Add
                </button>
            </div>
            <label htmlFor="selectCamera" className="font-medium">Select Device:</label>
            <select
                id="selectCamera"
                className="p-2 border rounded-md"
                value={selectedCamera?.id || ''}
                onChange={e => {
                    const camId = e.target.value;
                    let cam = null;
                    for (let i = 0; i < dynamicCameras.length; i++) {
                        if (dynamicCameras[i].id === camId) {
                            cam = dynamicCameras[i];
                            setSelectedCamera(cam);
                            if (cam.features && cam.features.length > 0) {
                                setSelectedFeatureGroup(cam.features[0].name);
                            } else {
                                setSelectedCamera(null);
                                setSelectedFeatureGroup(null);
                            }
                            break;
                        }
                    }
                }}
            >
                <option value="">-- Select a Device --</option>
                {dynamicCameras.map(cam => (
                    <option key={cam.id} value={cam.id}>
                        {cam.identifier} ({cam.type})
                    </option>
                ))}
            </select>
            {selectedCamera && (
                <>
                    <div className={`w-3 h-3 rounded-full ${selectedCamera.status === 'running' ? 'bg-green-500' : 'bg-red-500'}`}></div>
                    <span className="text-sm text-black">Temp: N/A</span>
                    <button
                        onClick={() => selectedCamera.apiEndpoint && handleAction(`${selectedCamera.apiEndpoint}/stop`, `${selectedCamera.identifier} driver stopped.`, `Failed to stop ${selectedCamera.identifier} driver`, selectedCamera)}
                        className="bg-red-500 hover:bg-red-700 text-white font-bold py-2 px-4 rounded disabled:opacity-50 disabled:cursor-not-allowed"
                        disabled={selectedCamera.status === "stopped" || !selectedCamera.apiEndpoint}
                    >
                        Stop
                    </button>
                    <button
                        onClick={() => selectedCamera.apiEndpoint && handleAction(`${selectedCamera.apiEndpoint}/start`, `${selectedCamera.identifier} publishing started.`, `Failed to start ${selectedCamera.identifier} publishing`, selectedCamera)}
                        className="bg-purple-500 hover:bg-purple-700 text-white font-bold py-2 px-4 rounded disabled:opacity-50 disabled:cursor-not-allowed"
                        disabled={selectedCamera.status === "running" || !selectedCamera.apiEndpoint}
                    >
                        Publish
                    </button>
                </>
            )}
        </div>
    );
};

export default DeviceSelectorPanel;
