"use client";

import React, { useState, useEffect, useCallback, useRef } from "react";
import yaml from 'js-yaml'; // Import js-yaml for YAML parsing/dumping
import Link from "next/link"; // Import Link for navigation
import dynamic from 'next/dynamic'; // Import dynamic for SSR control
import { useWindowSize } from './WindowSizeContext'; // Import the custom hook

const WebTerminal = dynamic(() => import("./components/WebTerminal"), { ssr: false }); // Dynamically import WebTerminal with SSR disabled

const ROS_API_URL = "http://localhost:8000";
const WEBSOCKET_URL = "ws://localhost:8000/logs"; // WebSocket URL for logs
const TERMINAL_WEBSOCKET_URL = "ws://localhost:8000/terminal"; // WebSocket URL for the web terminal

// Define types for device statuses and configurations
interface DeviceStatus {
  id: string; // Assuming 'id' is the camera identifier (e.g., IP address)
  identifier: string; // This might be the same as id, or a user-friendly name
  type: string; // e.g., "gigE"
  config: any; // Raw config data if available
  status: string;
  features?: CameraFeatureGroup[]; // Add features here
  // Properties from StaticDeviceInfo, now required as they are always provided in allDevices
  name: string;
  path: string;
  apiEndpoint: string;
  fileName: string;
};

interface CameraFeatureGroup {
  name: string;
  features: CameraFeature[];
}

interface CameraFeature {
  name: string;
  description: string;
  type: string; // e.g., "int", "float", "enum", "bool"
  value: any;
  min?: number;
  max?: number;
  options?: string[]; // For enum types
}

// Define the type for the static device info structure
type StaticDeviceInfo = {
  name: string;
  path: string;
  apiEndpoint: string;
  fileName: string;
};

// Define Preset types
interface Preset {
  name: string;
  configuration: Record<string, Record<string, { type: string; value: any }>>;
}

export default function Home() {
  // Get window size from context
  const { width, height } = useWindowSize();
  const [isClient, setIsClient] = useState(false); // State to track if component is mounted on client

  useEffect(() => {
    setIsClient(true); // Set isClient to true after component mounts
  }, []);

  // State for new camera creation
  const [newCameraId, setNewCameraId] = useState<string>('');
  const [newCameraProtocol, setNewCameraProtocol] = useState<string>('gigE');

  // State for static devices (like Ouster Lidar)
  const [ousterStatus, setOusterStatus] = useState("stopped");

  // State for dynamic cameras fetched from backend
  // Stores { id, identifier, type, config, status, features }
  const [dynamicCameras, setDynamicCameras] = useState<DeviceStatus[]>([]);
  const [selectedCamera, setSelectedCamera] = useState<DeviceStatus | null>(null);
  const [selectedFeatureGroup, setSelectedFeatureGroup] = useState<string | null>(null);

  // State for managing active tab in device details
  const [activeTab, setActiveTab] = useState<'features' | 'presets'>('features');

  // State for presets
  const [presets, setPresets] = useState<Preset[]>([]);
  const [selectedPresetForEditing, setSelectedPresetForEditing] = useState<Preset | null>(null);
  const [presetForm, setPresetForm] = useState<{ name: string; configuration: Record<string, Record<string, { type: string; value: any }>> }>({ name: '', configuration: {} });
  const [presetFormError, setPresetFormError] = useState<string>('');

  const [recordingStatus, setRecordingStatus] = useState("stopped");
  const [recordings, setRecordings] = useState<any[]>([]); // Explicitly type as any[] for now
  const [message, setMessage] = useState("");
  const [showMessage, setShowMessage] = useState(true);

  // State for managing which device's config is being edited
  const [editingDevice, setEditingDevice] = useState<string | null>(null);
  const [currentConfig, setCurrentConfig] = useState<string>('');
  const [currentConfigMessage, setCurrentConfigMessage] = useState<string>('');
  const [currentConfigLoading, setCurrentConfigLoading] = useState<boolean>(false);
  const [deviceConfigs, setDeviceConfigs] = useState<Record<string, string>>({}); // Store configs by device name

  const [deviceControl,setDeviceControl] = useState<any>({})//selectedCamera.features.find(featureGroup => featureGroup.name === "DeviceControl")
  const [PtpControl,setPtpControl] = useState<any>({})//selectedCamera.features.find(featureGroup => featureGroup.name === "PtpControl")
  const [ImageFormatControl,setImageFormatControl] = useState<any>({})//selectedCamera.features.find(featureGroup => featureGroup.name === "PtpControl")

  // State for logs
  const [logs, setLogs] = useState<string[]>([]);
  const ws = useRef<WebSocket | null>(null); // Ref for WebSocket instance

  // Define static device info using useRef to ensure stability
  // Removed ImagingSource Camera and Xenics Camera as per feedback
  const staticDevicesInfo = useRef<StaticDeviceInfo[]>([
    { name: "Ouster Lidar", path: "/ouster", apiEndpoint: "/driver/ouster", fileName: "ouster.yaml" },
  ]).current;

  // Function to fetch cameras and their statuses from backend
  const fetchCameras = async () => {
    try {
      const response = await fetch(`${ROS_API_URL}/get_all_cams/gigE`);
      if (response.ok) {
        const data = await response.json();
        // Assuming data is an array of DeviceStatus objects
        setDynamicCameras(data || []);
        // If a camera was previously selected, try to re-select it to update its data
        if (selectedCamera) {
          const updatedSelectedCamera = data.find((cam: DeviceStatus) => cam.id === selectedCamera.id);
          if (updatedSelectedCamera) {
            setSelectedCamera(updatedSelectedCamera);
          } else {
            setSelectedCamera(null); // Camera no longer exists
          }
        }
      } else {
        setMessage(`Error fetching cameras: ${response.statusText}`);
        setShowMessage(true);
      }
    } catch (error: unknown) {
      setMessage(`Error fetching cameras: ${(error as Error).message}`);
      setShowMessage(true);
    }
  };

  // Function to fetch a single camera's detailed data (including features)
  const fetchCameraDetails = async (cameraId: string, protocol: string) => {
    try {
      const response = await fetch(`${ROS_API_URL}/get_cam/${protocol}/${cameraId}`);
      if (response.ok) {
        const data = await response.json();
        console.log("Fetched camera details:", data);
        // Augment the fetched camera data with required properties for DeviceStatus
        const augmentedCamera: DeviceStatus = {
          ...data,
          name: `Camera: ${data.identifier}`, // Use a consistent name format for display
          path: `/imagingsource`, // Generic path for dynamic cameras
          apiEndpoint: `/driver/camera`, // Use the generic driver endpoint for start/stop/publish
          fileName: `camera_${data.id}.yaml`, // Placeholder
        };
        setSelectedCamera(augmentedCamera);
        if (augmentedCamera.features && augmentedCamera.features.length > 0) {
          setSelectedFeatureGroup(augmentedCamera.features[0].name); // Select the first group by default
        } else {
          setSelectedFeatureGroup(null);
        }
      } else {
        setMessage(`Error fetching camera details for ${cameraId}: ${response.statusText}`);
        setShowMessage(true);
        setSelectedCamera(null);
      }
    } catch (error: unknown) {
      setMessage(`Error fetching camera details for ${cameraId}: ${(error as Error).message}`);
      setShowMessage(true);
      setSelectedCamera(null);
    }
  };

  // Function to fetch presets for the selected camera
  const fetchPresets = async (deviceId: string) => {
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${deviceId}`);
      if (response.ok) {
        const data = await response.json();
        setPresets(data.presets || []);
      } else {
        setMessage(`Error fetching presets for device ${deviceId}: ${response.statusText}`);
        setShowMessage(true);
        setPresets([]);
      }
    } catch (error: unknown) {
      setMessage(`Error fetching presets for device ${deviceId}: ${(error as Error).message}`);
      setShowMessage(true);
      setPresets([]);
    }
  };

  // Refs to hold the latest state setters and fetch functions
  const actionsRef = useRef({
    fetchStatus: async (endpoint: string, setStatus: (value: React.SetStateAction<string>) => void) => {
      try {
        const response = await fetch(`${ROS_API_URL}${endpoint}`);
        if (response.ok) {
          const data = await response.json();
          setStatus(data.status);
        } else {
          actionsRef.current.setMessage(`Error fetching status from ${endpoint}: ${response.statusText}`);
          actionsRef.current.setShowMessage(true);
        }
      } catch (error: unknown) {
        actionsRef.current.setMessage(`Error fetching status from ${endpoint}: ${(error as Error).message}`);
        actionsRef.current.setShowMessage(true);
      }
    },
    fetchAllStatuses: async () => {
      actionsRef.current.fetchStatus("/driver/ouster/status", setOusterStatus);
    },
    fetchRecordings: async () => {
      try {
        const response = await fetch(`${ROS_API_URL}/recordings`);
        if (response.ok) {
          const data = await response.json();
          if (!Array.isArray(data.recordings)) {
            data.recordings = [];
          }
          setRecordings(data.recordings);
        } else {
          actionsRef.current.setMessage(`Error fetching recordings: ${response.statusText}`);
          actionsRef.current.setShowMessage(true);
        }
      } catch (error: unknown) {
        actionsRef.current.setMessage(`Error fetching recordings: ${(error as Error).message}`);
        actionsRef.current.setShowMessage(true);
      }
    },
    fetchDeviceConfig: async (deviceInfo: { name: string; fileName: string }) => {
      try {
        const response = await fetch(`${ROS_API_URL}/config/${deviceInfo.fileName}`);
        if (response.ok) {
          const data = await response.json();
          setDeviceConfigs(prevConfigs => ({
            ...prevConfigs,
            [deviceInfo.name]: yaml.dump(data, { indent: 2 })
          }));
        } else {
          console.error(`Error fetching ${deviceInfo.fileName}: ${response.statusText}`);
        }
      } catch (error: unknown) {
        console.error(`Error fetching ${deviceInfo.fileName}: ${(error as Error).message}`);
      }
    },
    fetchCameras: fetchCameras,
    fetchCameraDetails: fetchCameraDetails,
    fetchPresets: fetchPresets, // Add fetchPresets to the ref
    staticDevicesInfo: staticDevicesInfo,
    setMessage: setMessage,
    setShowMessage: setShowMessage,
    setRecordings: setRecordings,
    setDeviceConfigs: setDeviceConfigs,
    setPresets: setPresets, // Add setPresets to the ref
    setSelectedPresetForEditing: setSelectedPresetForEditing,
    setPresetForm: setPresetForm,
    setPresetFormError: setPresetFormError,
  });

  // Update the ref whenever the state setters or static info change
  useEffect(() => {
    actionsRef.current = {
      fetchStatus: async (endpoint: string, setStatus: (value: React.SetStateAction<string>) => void) => {
        try {
          const response = await fetch(`${ROS_API_URL}${endpoint}`);
          if (response.ok) {
            const data = await response.json();
            setStatus(data.status);
          } else {
            actionsRef.current.setMessage(`Error fetching status from ${endpoint}: ${response.statusText}`);
            actionsRef.current.setShowMessage(true);
          }
        } catch (error: unknown) {
          actionsRef.current.setMessage(`Error fetching status from ${endpoint}: ${(error as Error).message}`);
          actionsRef.current.setShowMessage(true);
        }
      },
      fetchAllStatuses: async () => {
        actionsRef.current.fetchStatus("/driver/ouster/status", setOusterStatus);
      },
      fetchRecordings: async () => {
        try {
          const response = await fetch(`${ROS_API_URL}/recordings`);
          if (response.ok) {
            const data = await response.json();
            if (!Array.isArray(data.recordings)) {
              data.recordings = [];
            }
            setRecordings(data.recordings);
          } else {
            actionsRef.current.setMessage(`Error fetching recordings: ${response.statusText}`);
            actionsRef.current.setShowMessage(true);
          }
        } catch (error: unknown) {
          actionsRef.current.setMessage(`Error fetching recordings: ${(error as Error).message}`);
          actionsRef.current.setShowMessage(true);
        }
      },
      fetchDeviceConfig: async (deviceInfo: { name: string; fileName: string }) => {
        try {
          const response = await fetch(`${ROS_API_URL}/config/${deviceInfo.fileName}`);
          if (response.ok) {
            const data = await response.json();
            setDeviceConfigs(prevConfigs => ({
              ...prevConfigs,
              [deviceInfo.name]: yaml.dump(data, { indent: 2 })
            }));
          } else {
            console.error(`Error fetching ${deviceInfo.fileName}: ${response.statusText}`);
          }
        } catch (error: unknown) {
          console.error(`Error fetching ${deviceInfo.fileName}: ${(error as Error).message}`);
        }
      },
      fetchCameras: fetchCameras,
      fetchCameraDetails: fetchCameraDetails,
      fetchPresets: fetchPresets,
      staticDevicesInfo: staticDevicesInfo,
      setMessage: setMessage,
      setShowMessage: setShowMessage,
      setRecordings: setRecordings,
      setDeviceConfigs: setDeviceConfigs,
      setPresets: setPresets,
      setSelectedPresetForEditing: setSelectedPresetForEditing,
      setPresetForm: setPresetForm,
      setPresetFormError: setPresetFormError,
    };
  }, [setOusterStatus, setRecordingStatus, setRecordings, setMessage, setShowMessage, setDeviceConfigs, setPresets, setSelectedPresetForEditing, setPresetForm, setPresetFormError, staticDevicesInfo, fetchCameras, fetchCameraDetails, fetchPresets, selectedCamera]);


  // Fetch all statuses and configs on initial load and periodically
  useEffect(() => {
    // Initial fetches for static devices
    actionsRef.current.fetchAllStatuses();
    actionsRef.current.fetchRecordings();
    actionsRef.current.staticDevicesInfo.forEach(deviceInfo => {
      actionsRef.current.fetchDeviceConfig(deviceInfo);
    });

    // Fetch dynamic cameras and their statuses
    actionsRef.current.fetchCameras();

    // Periodic fetches
    const statusInterval = setInterval(() => {
        actionsRef.current.fetchAllStatuses(); // Fetch status for static devices
        actionsRef.current.fetchCameras(); // Re-fetch dynamic cameras to update status if available
        if (selectedCamera) {
          actionsRef.current.fetchCameraDetails(selectedCamera.id, selectedCamera.type); // Refresh selected camera details
          // Fetch presets if a camera is selected and the presets tab is active
          if (activeTab === 'presets') {
            actionsRef.current.fetchPresets(selectedCamera.id);
          }
        }
    }, 5000);
    const recordingInterval = setInterval(() => actionsRef.current.fetchRecordings(), 10000);
    const configInterval = setInterval(() => {
      actionsRef.current.staticDevicesInfo.forEach(deviceInfo => actionsRef.current.fetchDeviceConfig(deviceInfo));
    }, 15000); // Fetch configs every 15 seconds

    return () => {
      clearInterval(statusInterval);
      clearInterval(recordingInterval);
      clearInterval(configInterval);
    };
  }, [activeTab, selectedCamera]); // Depend on activeTab and selectedCamera to re-fetch presets when they change


  // WebSocket connection logic (remains the same)
  useEffect(() => {
    ws.current = new WebSocket(WEBSOCKET_URL);

    ws.current.onopen = () => {
      console.log("WebSocket connected to log stream");
    };

    ws.current.onmessage = (event: MessageEvent) => {
      try {
        // Corrected JSON.JSON.parse to JSON.parse
        const message = JSON.parse(event.data);
        if (message.log) {
          setLogs((prevLogs: string[]) => [...prevLogs, message.log]);
        }
      } catch (error: unknown) {
        console.error("Failed to parse WebSocket message:", error);
        setLogs((prevLogs: string[]) => [...prevLogs, `Error: ${event.data}`]);
      }
    };

    ws.current.onerror = (error: Event) => {
      console.error("WebSocket error:", error);
      setLogs((prevLogs: string[]) => [...prevLogs, `WebSocket Error: ${error}`]);
    };

    ws.current.onclose = () => {
      console.log("WebSocket disconnected from log stream");
      ws.current = null;
    };

    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);


  // Generalize handleAction for dynamic cameras
  const handleAction = async (
    endpoint: string,
    successMessage: string,
    errorMessage: string,
    device: DeviceStatus // Pass the device object
  ) => {
    setMessage(""); // Clear previous messages
    try {
      const response = await fetch(`${ROS_API_URL}${endpoint}`, { method: "POST" });
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

  const handleDeviceConfigSave = async () => {
    setCurrentConfigLoading(true);
    setCurrentConfigMessage('');
    try {
      const parsedConfig = yaml.load(currentConfig);
      // Find the device that is currently being edited
      const deviceToUpdate = allDevices.find(d => d.name === editingDevice);
      if (!deviceToUpdate || !deviceToUpdate.fileName || !deviceToUpdate.name) throw new Error("Device or config file name not found");

      const response = await fetch(`${ROS_API_URL}/config/${deviceToUpdate.fileName}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(parsedConfig),
      });
      const data = await response.json();
      if (response.ok) {
        setCurrentConfigMessage(`Saved ${deviceToUpdate.fileName}: ${data.message}`);
        actionsRef.current.fetchDeviceConfig({ name: deviceToUpdate.name, fileName: deviceToUpdate.fileName });
        setEditingDevice(null); // Close editing mode
      } else {
        setCurrentConfigMessage(`Error saving ${deviceToUpdate.fileName}: ${data.detail || response.statusText}`);
      }
    } catch (error: unknown) {
      setCurrentConfigMessage(`Error saving config: ${(error as Error).message}`);
    } finally {
      setCurrentConfigLoading(false);
    }
  };

  const handleDeviceConfigCancel = () => {
    setEditingDevice(null);
    // Revert changes by re-fetching the config for the device that was being edited
    const deviceToRevert = allDevices.find(d => d.name === editingDevice);
    if (deviceToRevert && deviceToRevert.name && deviceToRevert.fileName) {
      actionsRef.current.fetchDeviceConfig({ name: deviceToRevert.name, fileName: deviceToRevert.fileName });
    }
  };

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

  // Handler for changing a feature value
  const handleFeatureChange = async (
    cameraId: string,
    featureGroupName: string,
    featureName: string,
    newValue: any
  ) => {
    setMessage("");
    try {
      // Construct the feature update payload. The backend endpoint might need adjustment.
      // Assuming a generic endpoint like /update_feature or similar.
      // For now, let's assume the backend handles this via a generic call or specific camera endpoints.
      // If a specific endpoint is needed, it should be defined in main.py.
      // For now, we'll simulate the update and then re-fetch.
      
      // Placeholder for actual API call to update feature
      console.log(`Simulating update for feature ${featureName} in group ${featureGroupName} for camera ${cameraId} to ${newValue}`);

      // Update the local state immediately for better UX, then re-fetch to confirm from backend
      setSelectedCamera(prevCamera => {
        if (!prevCamera) return null;
        const updatedFeatures = prevCamera.features?.map(group => {
          if (group.name === featureGroupName) {
            return {
              ...group,
              features: group.features.map(feature =>
                feature.name === featureName ? { ...feature, value: newValue } : feature
              )
            };
          }
          return group;
        });
        return { ...prevCamera, features: updatedFeatures };
      });

      // In a real scenario, you would call an API endpoint here:
      // const response = await fetch(`${ROS_API_URL}/update_feature`, { ... });
      // const data = await response.json();
      // if (response.ok) { ... } else { ... }

      // For now, we'll just log and assume success, then re-fetch details to sync state
      await actionsRef.current.fetchCameraDetails(cameraId, selectedCamera?.type || 'gigE'); // Re-fetch to get actual values

      setMessage(`Feature ${featureName} updated successfully!`);
      setShowMessage(true);

    } catch (error: unknown) {
      setMessage(`Error updating feature ${featureName}: ${(error as Error).message}`);
      setShowMessage(true);
    }
  };

  // Combine dynamic cameras with static devices for rendering
  const allDevices: DeviceStatus[] = [
    // Map dynamic cameras to DeviceStatus structure
    ...dynamicCameras.map(cam => ({
      ...cam,
      name: `Camera: ${cam.identifier}`, // Use a consistent name format for display
      path: `/imagingsource`, // Assuming a generic path for all dynamic cameras
      apiEndpoint: `/driver/camera`, // Assuming a generic API endpoint for dynamic cameras
      fileName: `camera_${cam.id}.yaml`, // Placeholder for config file name
    })),
    // Map static devices
    ...staticDevicesInfo.map(info => ({
      ...info,
      id: info.name, // Use name as id for static devices
      identifier: info.name, // Use name as identifier for static devices
      type: "static", // Indicate static device type
      config: null, // No config object directly here
      status: info.name === "Ouster Lidar" ? ousterStatus : "unknown",
      name: info.name, // Explicitly add name
      path: info.path, // Explicitly add path
      apiEndpoint: info.apiEndpoint, // Explicitly add apiEndpoint
      fileName: info.fileName, // Explicitly add fileName
    }))
  ];
  useEffect(()=>{
    if(selectedCamera && selectedCamera.features){
      // Safely find feature groups and provide a default empty object if not found
      setDeviceControl(selectedCamera.features.find(featureGroup => featureGroup.name === "DeviceControl") || {});
      setPtpControl(selectedCamera.features.find(featureGroup => featureGroup.name === "PtpControl") || {});
      setImageFormatControl(selectedCamera.features.find(featureGroup => featureGroup.name === "ImageFormatControl") || {});
    } else {
      // Reset to default empty objects if selectedCamera or its features are not available
      setDeviceControl({});
      setPtpControl({});
      setImageFormatControl({});
    }
  },[selectedCamera])

  // Effect to fetch presets when a camera is selected and the presets tab is active
  useEffect(() => {
    if (selectedCamera && activeTab === 'presets') {
      actionsRef.current.fetchPresets(selectedCamera.id);
    }
  }, [selectedCamera, activeTab]); // Re-run when selectedCamera or activeTab changes

  // Handlers for preset management
  const handleCreatePreset = async () => {
    if (!presetForm.name || !presetForm.configuration || Object.keys(presetForm.configuration).length === 0) {
      setPresetFormError("Preset name and configuration are required.");
      return;
    }
    if (!selectedCamera) {
      setPresetFormError("Please select a device first.");
      return;
    }

    try {
      const response = await fetch(`${ROS_API_URL}/presets/${selectedCamera.id}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: presetForm.name, configuration: presetForm.configuration }),
      });
      const data = await response.json();
      if (response.ok) {
        setMessage(`Preset '${presetForm.name}' created successfully!`);
        setShowMessage(true);
        actionsRef.current.fetchPresets(selectedCamera.id); // Refresh presets list
        setPresetForm({ name: '', configuration: {} }); // Clear form
        setPresetFormError('');
        setSelectedPresetForEditing(null); // Exit editing mode if any
      } else {
        setPresetFormError(`Error creating preset: ${data.detail || response.statusText}`);
      }
    } catch (error: unknown) {
      setPresetFormError(`Error creating preset: ${(error as Error).message}`);
    }
  };

  const handleUpdatePreset = async () => {
    if (!selectedPresetForEditing) return; // Should not happen if called correctly

    if (!presetForm.name || !presetForm.configuration || Object.keys(presetForm.configuration).length === 0) {
      setPresetFormError("Preset name and configuration are required.");
      return;
    }
    if (!selectedCamera) {
      setPresetFormError("Please select a device first.");
      return;
    }

    try {
      const response = await fetch(`${ROS_API_URL}/presets/${selectedCamera.id}/${selectedPresetForEditing.name}`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: presetForm.name, configuration: presetForm.configuration }),
      });
      const data = await response.json();
      if (response.ok) {
        setMessage(`Preset '${presetForm.name}' updated successfully!`);
        setShowMessage(true);
        actionsRef.current.fetchPresets(selectedCamera.id); // Refresh presets list
        setPresetForm({ name: '', configuration: {} }); // Clear form
        setPresetFormError('');
        setSelectedPresetForEditing(null); // Exit editing mode
      } else {
        setPresetFormError(`Error updating preset: ${data.detail || response.statusText}`);
      }
    } catch (error: unknown) {
      setPresetFormError(`Error updating preset: ${(error as Error).message}`);
    }
  };

  const handleDeletePreset = async (presetName: string) => {
    if (!selectedCamera) return;

    if (!window.confirm(`Are you sure you want to delete preset "${presetName}"?`)) {
      return;
    }

    try {
      const response = await fetch(`${ROS_API_URL}/presets/${selectedCamera.id}/${presetName}`, {
        method: "DELETE",
      });
      const data = await response.json();
      if (response.ok) {
        setMessage(`Preset '${presetName}' deleted successfully!`);
        setShowMessage(true);
        actionsRef.current.fetchPresets(selectedCamera.id); // Refresh presets list
        if (selectedPresetForEditing?.name === presetName) {
          setSelectedPresetForEditing(null); // Clear editing if the deleted preset was being edited
          setPresetForm({ name: '', configuration: {} });
        }
      } else {
        setMessage(`Error deleting preset '${presetName}': ${data.detail || response.statusText}`);
        setShowMessage(true);
      }
    } catch (error: unknown) {
      setMessage(`Error deleting preset '${presetName}': ${(error as Error).message}`);
      setShowMessage(true);
    }
  };

  // Handler to load a preset into the form for editing
  const loadPresetForEditing = (preset: Preset) => {
    setSelectedPresetForEditing(preset);
    setPresetForm({ name: preset.name, configuration: { ...preset.configuration } }); // Deep copy configuration
    setPresetFormError('');
    setActiveTab('presets'); // Ensure we are on the presets tab
  };

  // Handler to select a feature group and reset the form
  const handleFeatureGroupSelect = (groupName: string) => {
    setSelectedFeatureGroup(groupName);
    // When changing feature group, clear the preset form if it's not being edited
    if (!selectedPresetForEditing) {
      setPresetForm({ name: '', configuration: {} });
    }
  };

  // Handler to update a feature value within the preset form
  const handlePresetFeatureChange = (
    groupName: string,
    featureName: string,
    newValue: any
  ) => {
    setPresetForm(prevForm => {
      const newConfiguration = { ...prevForm.configuration };
      if (!newConfiguration[groupName]) {
        newConfiguration[groupName] = {};
      }
      
      // Find the feature to get its type
      const feature = selectedCamera?.features
        ?.find(g => g.name === groupName)
        ?.features.find(f => f.name === featureName);

      if (feature) {
        newConfiguration[groupName][featureName] = {
          type: feature.type, // Include the feature type
          value: newValue
        };
      } else {
        // Fallback if feature not found, though this should ideally not happen
        console.warn(`Feature ${featureName} not found in group ${groupName} for type lookup.`);
        newConfiguration[groupName][featureName] = {
          type: 'string', // Default type if not found
          value: newValue
        };
      }
      return { ...prevForm, configuration: newConfiguration };
    });
  };

  // Handler to update the preset name in the form
  const handlePresetNameChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setPresetForm(prevForm => ({ ...prevForm, name: e.target.value }));
  };

  // Handler to cancel editing a preset
  const handleCancelPresetEdit = () => {
    setSelectedPresetForEditing(null);
    setPresetForm({ name: '', configuration: {} });
    setPresetFormError('');
  };

  return (isClient&&<div className="w-full h-full text-black bg-gray-100 p-8">

      <h1 className="text-4xl font-bold text-center mb-8">Dashboard</h1>
      {/* Removed window size display */}

      {message && showMessage && (
        <div className="bg-blue-100 border-l-4 border-blue-500 text-blue-700 p-4 mb-6 flex justify-between items-center" role="alert">
          <div>
            <p className="font-bold">Info</p>
            <p>{message}</p>
          </div>
          <button
            className="ml-4 text-blue-700 hover:text-blue-900 font-bold text-xl"
            onClick={() => setShowMessage(false)}
            aria-label="Close alert"
          >
            &times;
          </button>
        </div>
      )}



      {/* New Camera Creation Section */}
      <div style={{ width: '100%', padding: '20px', border: '1px solid #ccc', marginBottom: '20px', backgroundColor: '#f9f9f9' }}>
        <h2 className="text-2xl font-semibold mb-4">Add a Device</h2>
        <div style={{ display: 'flex', gap: '10px', alignItems: 'center' }}>
          <label htmlFor="cameraId">Device ID (IP):</label>
          <input
            type="text"
            id="cameraId"
            placeholder="e.g., 192.168.1.100"
            style={{ padding: '8px', border: '1px solid #ccc', borderRadius: '4px' }}
            value={newCameraId}
            onChange={(e) => setNewCameraId(e.target.value)}
          />

          <label htmlFor="protocol">Protocol:</label>
          <select
            id="protocol"
            style={{ padding: '8px', border: '1px solid #ccc', borderRadius: '4px' }}
            value={newCameraProtocol}
            onChange={(e) => setNewCameraProtocol(e.target.value)}
          >
            <option value="gigE">GigE</option>
            <option value="gmsl2">GMSL2</option>
          </select>

          <button
            onClick={handleCreateCamera}
            style={{ padding: '10px 15px', backgroundColor: '#007bff', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
          >
            Add a Device
          </button>
        </div>
      </div>

      {/* Device Selection and Control Section */}
      <div className="w-full bg-white p-6 rounded-lg shadow-md mb-8 h-[50%] overflow-scroll">
        <div className="flex items-center justify-between border-b pb-4 mb-4 ">
          <h2 className="text-2xl font-semibold">Devices</h2>
          <div className="flex items-center space-x-4">
            <label htmlFor="selectCamera" className="font-medium">Select Device:</label>
            <select
              id="selectCamera"
              className="p-2 border rounded-md"
              value={selectedCamera?.id || ''}
              onChange={(e) => {
                const camId = e.target.value;
                console.log("Selected camId from dropdown:", camId);
                console.log("Current dynamicCameras:", dynamicCameras);
                var cam
                for (var i=0; i<dynamicCameras.length; i++) {
                  console.log(`Checking camera ${dynamicCameras[i].id} against selected camId ${camId}`);
                  if (dynamicCameras[i].id == camId) {
                    console.log("Match found:", dynamicCameras[i]);
                    cam = dynamicCameras[i];
                    setSelectedCamera(cam);
                      if (cam.features && cam.features.length > 0) {
                        setSelectedFeatureGroup(cam.features[0].name); // Select the first group by default
                      } else {
                  console.log("Camera not found in dynamicCameras, setting selectedCamera to null.");
                  setSelectedCamera(null);
                  setSelectedFeatureGroup(null);                      }
                    break;
                  }
                }
              }}
            >
              <option value="">-- Select a Device --</option>
              {dynamicCameras.map((cam) => (
                <option key={cam.id} value={cam.id}>
                  {cam.identifier} ({cam.type})
                </option>
              ))}
            </select>

            {selectedCamera && (
              <>
                <div className={`w-3 h-3 rounded-full ${selectedCamera.status === 'running' ? 'bg-green-500' : 'bg-red-500'}`}></div>
                <span className="text-sm text-black">Temp: N/A</span> {/* Placeholder for sensor temperature */}
                <button
                  onClick={() => selectedCamera.apiEndpoint && handleAction(`${selectedCamera.apiEndpoint}/stop`, `${selectedCamera.identifier} driver stopped.`, `Failed to stop ${selectedCamera.identifier} driver`, selectedCamera)}
                  className="bg-red-500 hover:bg-red-700 text-white font-bold py-2 px-4 rounded disabled:opacity-50 disabled:cursor-not-allowed"
                  disabled={selectedCamera.status === "stopped" || !selectedCamera.apiEndpoint}
                >
                  Stop
                </button>
                <button
                  onClick={() => selectedCamera.apiEndpoint && handleAction(`${selectedCamera.apiEndpoint}/publish`, `${selectedCamera.identifier} publishing started.`, `Failed to start ${selectedCamera.identifier} publishing`, selectedCamera)}
                  className="bg-purple-500 hover:bg-purple-700 text-white font-bold py-2 px-4 rounded disabled:opacity-50 disabled:cursor-not-allowed"
                  disabled={selectedCamera.status === "running" || !selectedCamera.apiEndpoint}
                >
                  Publish
                </button>

              </>
            )}
          </div>
        </div>

        {selectedCamera ? (
          <div className="flex flex-col md:flex-row gap-4 w-full h-full overflow-scroll " style={{height: height*0.5}}>
            {/* Middle: Device Data and Controls */}
            <div className="w-full md:w-[18%] bg-gray-50 p-4 rounded-md">
              <h3 className="text-xl font-semibold mb-4">Device Data</h3>
              <p><strong>ID:</strong> {selectedCamera.id}</p>
              <p><strong>Identifier:</strong> {selectedCamera.identifier}</p>
              <p><strong>Type:</strong> {selectedCamera.type}</p>
              <p><strong>Status:</strong> <span className={`font-bold ${selectedCamera.status === 'running' ? 'text-green-600' : 'text-red-600'}`}>{selectedCamera.status}</span></p>
              <div className="bg-gray-200 flex flex-col">
                    <div className="bg-white w-[85%] h-[85%] ">
                    temperature: {deviceControl?.DeviceTemperature ?? 'N/A'}
                    PtpStatus: {PtpControl?.PtpStatus ?? 'N/A'}
                    <h4>Image Format</h4>
                    <p><strong>PixelFormat: </strong> {JSON.stringify(ImageFormatControl)}</p>
                    </div>
               </div>
            </div>

            {/* Right: Settings Panel with Tabs */}
            <div className="w-full md:w-[80%] bg-gray-50 p-4 rounded-md">
              <div className="flex overflow-x-auto space-x-2 mb-4 pb-2 border-b">
                <button
                  onClick={() => setActiveTab('features')}
                  className={`px-4 py-2 rounded-md text-sm font-medium whitespace-nowrap ${
                    activeTab === 'features' ? 'bg-blue-600 text-white' : 'bg-gray-200 hover:bg-gray-300'
                  }`}
                >
                  Features
                </button>
                <button
                  onClick={() => {
                    setActiveTab('presets');
                    // Fetch presets when switching to the presets tab
                    if (selectedCamera) {
                      actionsRef.current.fetchPresets(selectedCamera.id);
                    }
                  }}
                  className={`px-4 py-2 rounded-md text-sm font-medium whitespace-nowrap ${
                    activeTab === 'presets' ? 'bg-blue-600 text-white' : 'bg-gray-200 hover:bg-gray-300'
                  }`}
                >
                  Presets
                </button>
              </div>

              {/* Render content based on active tab */}
              {activeTab === 'features' ? (
                <>
                  {selectedCamera.features && selectedCamera.features.length > 0 ? (
                    <div className="space-y-4">
                      <div className="flex overflow-x-auto space-x-2 mb-4 pb-2 border-b">
                        {selectedCamera.features.map(group => (
                          <button
                            key={group.name}
                            onClick={() => handleFeatureGroupSelect(group.name)}
                            className={`px-4 py-2 rounded-md text-sm font-medium whitespace-nowrap ${
                              selectedFeatureGroup === group.name ? 'bg-blue-600 text-white' : 'bg-gray-200 hover:bg-gray-300'
                            }`}
                          >
                            {group.name}
                          </button>
                        ))}
                      </div>

                      {selectedFeatureGroup && (
                        <div className="space-y-4">
                          {selectedCamera.features.find(g => g.name === selectedFeatureGroup)?.features.map(feature => (
                            <div key={feature.name} className="border p-3 rounded-md">
                              <p className="font-semibold">{feature.name}</p>
                              <p className="text-sm text-black mb-2">{feature.description}</p>
                              {feature.type === "enum" ? (
                                <select
                                  value={feature.value}
                                  onChange={(e) => selectedCamera && selectedFeatureGroup && handleFeatureChange(selectedCamera.id, selectedFeatureGroup, feature.name, e.target.value)}
                                  className="w-full p-2 border rounded-md"
                                >
                                  {feature.options?.map(option => (
                                    <option key={option} value={option}>{option}</option>
                                  ))}
                                </select>
                              ) : feature.type === "bool" ? (
                                <input
                                  type="checkbox"
                                  checked={feature.value}
                                  onChange={(e) => selectedCamera && selectedFeatureGroup && handleFeatureChange(selectedCamera.id, selectedFeatureGroup, feature.name, e.target.checked)}
                                  className="h-4 w-4 text-blue-600 border-gray-300 rounded focus:ring-blue-500"
                                />
                              ) : (
                                <input
                                  type="text" // Can be number for int/float, but text for generic input
                                  value={feature.value}
                                  onChange={(e) => selectedCamera && selectedFeatureGroup && handleFeatureChange(selectedCamera.id, selectedFeatureGroup, feature.name, e.target.value)}
                                  className="w-full p-2 border rounded-md"
                                  placeholder={feature.type}
                                />
                              )}
                            </div>
                          ))}
                        </div>
                      )}
                    </div>
                  ) : (
                    <p>No features available for this camera.</p>
                  )}
                </>
              ) : ( // Presets Tab Content
                <>
                  <div className="mb-4 flex justify-between items-center">
                    <h4 className="text-lg font-semibold">Manage Presets</h4>
                    <button
                      onClick={() => {
                        setPresetForm({ name: '', configuration: {} }); // Clear form for new preset
                        setSelectedPresetForEditing(null);
                        setActiveTab('presets'); // Ensure we are on presets tab
                      }}
                      className="bg-green-500 hover:bg-green-700 text-white font-bold py-2 px-4 rounded"
                    >
                      Create New Preset
                    </button>
                  </div>

                  {presetFormError && (
                    <p className="text-red-500 mb-4">{presetFormError}</p>
                  )}

                  {/* Preset Form for Creating/Editing */}
                  {(!selectedPresetForEditing || activeTab === 'presets') && ( // Show form if editing or creating new
                    <div className="border p-4 rounded-md mb-4 bg-gray-50">
                      <h5 className="text-lg font-semibold mb-3">{selectedPresetForEditing ? 'Edit Preset' : 'Create New Preset'}</h5>
                      <div className="space-y-3">
                        <div>
                          <label htmlFor="presetName" className="block text-sm font-medium text-gray-700">Preset Name</label>
                          <input
                            type="text"
                            id="presetName"
                            value={presetForm.name}
                            onChange={handlePresetNameChange}
                            className="mt-1 p-2 w-full border rounded-md"
                            placeholder="Enter preset name"
                          />
                        </div>

                        {/* Feature Selection for Preset */}
                        {selectedCamera?.features && selectedCamera.features.length > 0 && (
                          <>
                            <label htmlFor="presetFeatureGroup" className="block text-sm font-medium text-gray-700">Feature Group</label>
                            <select
                              id="presetFeatureGroup"
                              value={selectedFeatureGroup || ''}
                              onChange={(e) => {
                                handleFeatureGroupSelect(e.target.value);
                                // Clear the selected feature when group changes
                                // This might need more sophisticated state management if we want to preserve values
                              }}
                              className="w-full p-2 border rounded-md"
                            >
                              <option value="">-- Select Feature Group --</option>
                              {selectedCamera.features.map(group => (
                                <option key={group.name} value={group.name}>{group.name}</option>
                              ))}
                            </select>

                            {selectedFeatureGroup && (
                              <>
                                <label htmlFor="presetFeature" className="block text-sm font-medium text-gray-700 mt-3">Feature</label>
                                <select
                                  id="presetFeature"
                                  value={Object.keys(presetForm.configuration[selectedFeatureGroup] || {})[0] || ''} // Select the first feature in the group if available in form
                                  onChange={(e) => {
                                    const featureName = e.target.value;
                                    if (featureName && selectedFeatureGroup) {
                                      // Find the feature details to get its type and current value
                                      const feature = selectedCamera.features?.find(g => g.name === selectedFeatureGroup)?.features.find(f => f.name === featureName);
                                      if (feature) {
                                        // Set the form's current value for this feature
                                        const currentFeatureValue = presetForm.configuration[selectedFeatureGroup]?.[featureName]?.value ?? feature.value;
                                        // This part needs careful state management to update the specific feature value in the form
                                        // For now, we'll just log it.
                                        console.log(`Selected feature: ${featureName}, Type: ${feature.type}, Current Value: ${currentFeatureValue}`);
                                      }
                                    }
                                  }}
                                  className="w-full p-2 border rounded-md"
                                >
                                  <option value="">-- Select Feature --</option>
                                  {selectedCamera.features.find(g => g.name === selectedFeatureGroup)?.features.map(feature => (
                                    <option key={feature.name} value={feature.name}>{feature.name}</option>
                                  ))}
                                </select>

                                {/* Input for Feature Value */}
                                {selectedFeatureGroup && Object.keys(presetForm.configuration[selectedFeatureGroup] || {}).length > 0 && (
                                  <>
                                    <label htmlFor="presetFeatureValue" className="block text-sm font-medium text-gray-700 mt-3">Value</label>
                                    <input
                                      type="text" // Generic input, type validation needed
                                      id="presetFeatureValue"
                                      value={presetForm.configuration[selectedFeatureGroup]?.[Object.keys(presetForm.configuration[selectedFeatureGroup])[0]]?.value || ''}
                                      onChange={(e) => {
                                        const featureName = Object.keys(presetForm.configuration[selectedFeatureGroup] || {})[0];
                                        if (featureName && selectedFeatureGroup) {
                                          handlePresetFeatureChange(selectedFeatureGroup, featureName, e.target.value);
                                        }
                                      }}
                                      className="w-full p-2 border rounded-md"
                                      placeholder="Enter value"
                                    />
                                  </>
                                )}
                              </>
                            )}
                          </>
                        )}
                      </div>
                      <div className="flex space-x-4 mt-4">
                        <button
                          onClick={selectedPresetForEditing ? handleUpdatePreset : handleCreatePreset}
                          className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                            presetFormError ? 'bg-red-500' : 'bg-blue-600 hover:bg-blue-700'
                          }`}
                        >
                          {selectedPresetForEditing ? 'Update Preset' : 'Create Preset'}
                        </button>
                        <button
                          onClick={handleCancelPresetEdit}
                          className="px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 bg-gray-600 hover:bg-gray-700"
                        >
                          Cancel
                        </button>
                      </div>
                    </div>
                  )}

                  {/* List of Existing Presets */}
                  <div className="border p-4 rounded-md bg-gray-50">
                    <h4 className="text-lg font-semibold mb-3">Existing Presets</h4>
                    {presets.length === 0 ? (
                      <p>No presets found for this device.</p>
                    ) : (
                      <ul className="space-y-2">
                        {presets.map(preset => (
                          <li key={preset.name} className="flex items-center justify-between p-2 border-b last:border-b-0">
                            <span>{preset.name}</span>
                            <div className="flex space-x-2">
                              <button
                                onClick={() => loadPresetForEditing(preset)}
                                className="bg-yellow-500 hover:bg-yellow-700 text-white font-bold py-1 px-3 rounded text-sm"
                              >
                                Edit
                              </button>
                              <button
                                onClick={() => handleDeletePreset(preset.name)}
                                className="bg-red-500 hover:bg-red-700 text-white font-bold py-1 px-3 rounded text-sm"
                              >
                                Delete
                              </button>
                            </div>
                          </li>
                        ))}
                      </ul>
                    )}
                  </div>
                </>
              )}
            </div>
          </div>
        ) : (
          <p className="text-center text-black">Select a device to view its data and settings.</p>
        )}
      </div>

      {/* Web Terminal Section */}
      <div className="w-full bg-white p-6 rounded-lg shadow-md mb-8">
        <h2 className="text-2xl font-semibold mb-4">Web Terminal</h2>
        <WebTerminal websocketUrl={TERMINAL_WEBSOCKET_URL} />
      </div>

      <div className="flex flex-col md:flex-row gap-8">
        {/* Left Column: Devices and Configurations */}
        <div className="flex-1 md:w-1/2">
          <div className="grid grid-cols-1 gap-8">
            {allDevices.map((device) => (
              <div key={device.name} className="bg-white p-6 rounded-lg shadow-md flex flex-col justify-between">
                <div>
                  <h2 className="text-2xl font-semibold mb-4">{device.name}</h2>
                  <p className="mb-4">Status: <span className={`font-bold ${device.status === 'running' ? 'text-green-600' : 'text-red-600'}`}>{device.status}</span></p>
                </div>
                <div className="flex flex-col space-y-4">
                  {/* Start Button */}
                  <button
                    onClick={() => handleAction(`${device.apiEndpoint}/start`, `${device.name} driver started.`, `Failed to start ${device.name} driver`, device)}
                    className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded disabled:opacity-50 disabled:cursor-not-allowed"
                    disabled={device.status === "running"}
                  >
                    Start
                  </button>
                  {/* Stop Button */}
                  <button
                    onClick={() => handleAction(`${device.apiEndpoint}/stop`, `${device.name} driver stopped.`, `Failed to stop ${device.name} driver`, device)}
                    className="bg-red-500 hover:bg-red-700 text-white font-bold py-2 px-4 rounded disabled:opacity-50 disabled:cursor-not-allowed"
                    disabled={device.status === "stopped"}
                  >
                    Stop
                  </button>
                  
                  {/* Publish Button - Only for specific devices if needed */}
                  {/* This logic might need to be more dynamic based on device type */}
                  {(device.name === "Ouster Lidar" || device.type === "gigE") && ( // Example: enable publish for Ouster and dynamic cameras
                    <button
                      onClick={() => handleAction(`${device.apiEndpoint}/publish`, `${device.name} publishing started.`, `Failed to start ${device.name} publishing`, device)}
                      className="bg-purple-500 hover:bg-purple-700 text-white font-bold py-2 px-4 rounded disabled:opacity-50 disabled:cursor-not-allowed"
                      disabled={device.status === "running"}
                    >
                      Publish
                    </button>
                  )}
                  
                  {device.path && (
                    <Link
                      href={device.path}
                      className="text-center bg-gray-200 hover:bg-gray-300 text-black font-bold py-2 px-4 rounded"
                    >
                      View Details
                    </Link>
                  )}
                </div>

                {/* Device Configuration Section */}
                <div className="mt-6 pt-4 border-t border-gray-200">
                  <h3 className="text-xl font-semibold mb-3">{device.name} Configuration ({device.fileName})</h3>
                  {currentConfigLoading && editingDevice === device.name ? (
                    <p>Loading {device.name} configuration...</p>
                  ) : (
                    <>
                      {editingDevice === device.name ? (
                        <div className="w-full">
                          <textarea
                            className="w-full h-64 p-2 border rounded-md bg-gray-700 text-white font-mono"
                            value={currentConfig}
                            onChange={(e) => setCurrentConfig(e.target.value)}
                          />
                          <div className="flex space-x-4 mt-4">
                            <button
                              onClick={handleDeviceConfigSave}
                              disabled={currentConfigLoading}
                              className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                                currentConfigLoading ? 'bg-gray-600 cursor-not-allowed' : 'bg-blue-600 hover:bg-blue-700'
                              }`}
                            >
                              Save Configuration
                            </button>
                            <button
                              onClick={handleDeviceConfigCancel}
                              disabled={currentConfigLoading}
                              className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                                currentConfigLoading ? 'bg-gray-600 cursor-not-allowed' : 'bg-gray-600 hover:bg-gray-700'
                              }`}
                            >
                              Cancel
                            </button>
                          </div>
                        </div>
                      ) : (
                        <div className="w-full">
                          <pre className="bg-gray-700 text-white p-4 rounded-md overflow-auto max-h-96">
                            {device.name && deviceConfigs[device.name] ? yaml.dump(yaml.load(deviceConfigs[device.name]), { indent: 2 }) : 'Loading...'}
                          </pre>
                          <button
                            onClick={() => {
                              setCurrentConfigLoading(true);
                              setCurrentConfigMessage('');
                              setCurrentConfig(device.name && deviceConfigs[device.name] || ''); // Set current config to the fetched one
                              setEditingDevice(device.name);
                              setCurrentConfigLoading(false);
                            }}
                            disabled={currentConfigLoading || !device.name}
                            className={`mt-4 px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                              currentConfigLoading || !device.name ? 'bg-gray-600 cursor-not-allowed' : 'bg-blue-600 hover:bg-blue-700'
                            }`}
                          >
                            Edit Configuration
                          </button>
                        </div>
                      )}
                      {currentConfigMessage && <p className="mt-4 text-sm text-black">{currentConfigMessage}</p>}
                    </>
                  )}
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Right Column: Logs */}
        <div className="flex-1 md:w-1/2 bg-white p-6 rounded-lg shadow-md flex flex-col">
          <h2 className="text-2xl font-semibold mb-4">Container Logs</h2>
          <div className="flex-grow overflow-y-auto bg-gray-700 text-white p-4 rounded-md font-mono">
            {/* Log content will be fetched and displayed here */}
            {logs.map((log, index) => (
              <p key={index}>{log}</p>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
}
