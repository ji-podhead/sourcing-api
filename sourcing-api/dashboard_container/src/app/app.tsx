"use client";

import React, { useState, useEffect, useCallback, useRef } from "react";
import yaml from 'js-yaml'; // Import js-yaml for YAML parsing/dumping
import Link from "next/link"; // Import Link for navigation
import dynamic from 'next/dynamic'; // Import dynamic for SSR control
import { useWindowSize } from './handlers/WindowSizeContext'; // Import the custom hook
import { useSelector, useDispatch } from 'react-redux';
import { RootState } from './redux/store'; // Ensure this is the correct import for your store type
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
} from './redux/dashboardSlice';
import { setSelectedCamera, setDynamicCameras, setOusterStatus } from './redux/devicesSlice';
import { DeviceStatus, CameraFeatureGroup, CameraFeature, StaticDeviceInfo, Preset } from './types';

const WebTerminal = dynamic(() => import("./components/WebTerminal"), { ssr: false }); // Dynamically import WebTerminal with SSR disabled
import Sidebar from "./components/Sidebar";
import FeaturePanel from "./components/FeaturePanel";
import PresetsPanel from "./components/PresetsPanel";
import DeviceSelectorPanel from "./components/DeviceSelectorPanel";
import Settings from "./components/Settings";
import DeviceDetailsPanel from "./components/DeviceDetailsPanel";


const ROS_API_URL = "http://localhost:8000";
const WEBSOCKET_URL = "ws://localhost:8000/logs";
const TERMINAL_WEBSOCKET_URL = "ws://localhost:8000/terminal"; // WebSocket URL for the web terminal

export default function App() {
  // Get window size from context
  const { width, height } = useWindowSize();
  const dispatch = useDispatch();
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
  const fetchPresets = async (deviceIdentifier: string) => {
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${deviceIdentifier}`);
      if (response.ok) {
        const data = await response.json();
        setPresets(data.presets || []);
      } else {
        setMessage(`Error fetching presets for device ${deviceIdentifier}: ${response.statusText}`);
        setShowMessage(true);
        setPresets([]);
      }
    } catch (error: unknown) {
      setMessage(`Error fetching presets for device ${deviceIdentifier}: ${(error as Error).message}`);
      setShowMessage(true);
      setPresets([]);
    }
  };

  // Refs to hold the latest state setters and fetch functions
  const actionsRef = useRef({
    fetchStatus: async (endpoint: string, protocol?: string, cameraId?: string): Promise<string> => {
      try {
        let url = `${ROS_API_URL}${endpoint}`;
        if (protocol && cameraId) {
          url += `?protocol=${protocol}&camera_id=${cameraId}`;
        }
        const response = await fetch(url);
        if (response.ok) {
          const data = await response.json();
          return data.status;
        } else {
          actionsRef.current.setMessage(`Error fetching status from ${endpoint}: ${response.statusText}`);
          actionsRef.current.setShowMessage(true);
          return "unknown"; // Return a default status on error
        }
      } catch (error: unknown) {
        actionsRef.current.setMessage(`Error fetching status from ${endpoint}: ${(error as Error).message}`);
        actionsRef.current.setShowMessage(true);
        return "unknown"; // Return a default status on error
      }
    },
      fetchAllStatuses: async () => {
        // Fetch status for static devices
        const ousterStatusResult = await actionsRef.current.fetchStatus("/driver/ouster/status");
        setOusterStatus(ousterStatusResult);

        // Fetch dynamic cameras first to get the latest list
        const currentDynamicCameras = [...dynamicCameras]; // Create a copy to avoid modifying state directly

        // Fetch status for dynamic cameras
        const updatedDynamicCameras = await Promise.all(
          currentDynamicCameras.map(async (cam) => {
            const status = await actionsRef.current.fetchStatus(
              `/driver/${cam.apiEndpoint.split('/').pop()}/status`,
              cam.type,
              cam.id
            );
            return { ...cam, status };
          })
        );

        // Update dynamicCameras state
        setDynamicCameras(updatedDynamicCameras);

        // If selected camera's status changed, update selectedCamera
        if (selectedCamera) {
          const updatedSelected = updatedDynamicCameras.find((c) => c.id === selectedCamera.id);
          if (updatedSelected && updatedSelected.status !== selectedCamera.status) {
            setSelectedCamera(updatedSelected);
          }
        }
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
    fetchStatus: async (endpoint: string, protocol?: string, cameraId?: string): Promise<string> => {
      try {
        let url = `${ROS_API_URL}${endpoint}`;
        if (protocol && cameraId) {
          url += `?protocol=${protocol}&camera_id=${cameraId}`;
        }
        const response = await fetch(url);
        if (response.ok) {
          const data = await response.json();
          return data.status;
        } else {
          actionsRef.current.setMessage(`Error fetching status from ${endpoint}: ${response.statusText}`);
          actionsRef.current.setShowMessage(true);
          return "unknown"; // Return a default status on error
        }
      } catch (error: unknown) {
        actionsRef.current.setMessage(`Error fetching status from ${endpoint}: ${(error as Error).message}`);
        actionsRef.current.setShowMessage(true);
        return "unknown"; // Return a default status on error
      }
    },
      fetchAllStatuses: async () => {
        // Fetch status for static devices
        const ousterStatusResult = await actionsRef.current.fetchStatus("/driver/ouster/status");
        setOusterStatus(ousterStatusResult);

        // Fetch dynamic cameras first to get the latest list
        const currentDynamicCameras = [...dynamicCameras]; // Create a copy to avoid modifying state directly

        // Fetch status for dynamic cameras
        const updatedDynamicCameras = await Promise.all(
          currentDynamicCameras.map(async (cam) => {
            const status = await actionsRef.current.fetchStatus(
              `/driver/${cam.apiEndpoint.split('/').pop()}/status`,
              cam.type,
              cam.id
            );
            return { ...cam, status };
          })
        );

        // Update dynamicCameras state
        setDynamicCameras(updatedDynamicCameras);

        // If selected camera's status changed, update selectedCamera
        if (selectedCamera) {
          const updatedSelected = updatedDynamicCameras.find((c) => c.id === selectedCamera.id);
          if (updatedSelected && updatedSelected.status !== selectedCamera.status) {
            setSelectedCamera(updatedSelected);
          }
        }
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
  }, [setOusterStatus, setDynamicCameras, dynamicCameras, selectedCamera, setRecordingStatus, setRecordings, setMessage, setShowMessage, setDeviceConfigs, setPresets, setSelectedPresetForEditing, setPresetForm, setPresetFormError, staticDevicesInfo, fetchCameras, fetchCameraDetails, fetchPresets]);


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
        
        // Fetch presets if a camera is selected and the presets tab is active
        if (selectedCameraRef.current && activeTabRef.current === 'presets') {
          actionsRef.current.fetchPresets(selectedCameraRef.current.identifier);
        }
    }, refreshInterval);
    const recordingInterval = setInterval(() => actionsRef.current.fetchRecordings(), 10000);
    const configInterval = setInterval(() => {
      actionsRef.current.staticDevicesInfo.forEach(deviceInfo => actionsRef.current.fetchDeviceConfig(deviceInfo));
    }, 15000); // Fetch configs every 15 seconds

    return () => {
      clearInterval(statusInterval);
      clearInterval(recordingInterval);
      clearInterval(configInterval);
    };
  }, [refreshInterval]); // Only depends on refreshInterval


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
      setAnalogControl(selectedCamera.features.find(featureGroup => featureGroup.name === "AnalogControl") || {});
      setAcquisitionControl(selectedCamera.features.find(featureGroup => featureGroup.name === "AcquisitionControl") || {});
    } else {
      // Reset to default empty objects if selectedCamera or its features are not available
      setDeviceControl({});
      setPtpControl({});
      setImageFormatControl({});
      setAnalogControl({});
      setAcquisitionControl({});
    }
  },[selectedCamera])

  // Effect to fetch presets when a camera is selected and the presets tab is active
  useEffect(() => {
    if (selectedCamera && activeTab === 'presets') {
      actionsRef.current.fetchPresets(selectedCamera.identifier);
    }
  }, [selectedCamera, activeTab]); // Re-run when selectedCamera or activeTab changes

  useEffect(() => {
    activeTabRef.current = activeTab;
    selectedCameraRef.current = selectedCamera;
  });

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
      const response = await fetch(`${ROS_API_URL}/presets/${selectedCamera.identifier}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: presetForm.name, configuration: presetForm.configuration }),
      });
      const data = await response.json();
      if (response.ok) {
        setMessage(`Preset '${presetForm.name}' created successfully!`);
        setShowMessage(true);
        actionsRef.current.fetchPresets(selectedCamera.identifier); // Refresh presets list
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
      const response = await fetch(`${ROS_API_URL}/presets/${selectedCamera.identifier}/${selectedPresetForEditing.name}`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: presetForm.name, configuration: presetForm.configuration }),
      });
      const data = await response.json();
      if (response.ok) {
        setMessage(`Preset '${presetForm.name}' updated successfully!`);
        setShowMessage(true);
        actionsRef.current.fetchPresets(selectedCamera.identifier); // Refresh presets list
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
      const response = await fetch(`${ROS_API_URL}/presets/${selectedCamera.identifier}/${presetName}`, {
        method: "DELETE",
      });
      const data = await response.json();
      if (response.ok) {
        setMessage(`Preset '${presetName}' deleted successfully!`);
        setShowMessage(true);
        actionsRef.current.fetchPresets(selectedCamera.identifier); // Refresh presets list
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

  // Handler to update a feature's value in the preset form
  const handlePresetValueChange = (groupName: string, featureName: string, newValue: any) => {
    setPresetForm(prevForm => {
      const newConfig = JSON.parse(JSON.stringify(prevForm.configuration));
      if (newConfig[groupName] && newConfig[groupName][featureName]) {
        newConfig[groupName][featureName].value = newValue;
      }
      return { ...prevForm, configuration: newConfig };
    });
  };

  // Handler to toggle a feature's inclusion in the preset
  const handleTogglePresetFeature = (feature: CameraFeature, groupName: string, isChecked: boolean) => {
    setPresetForm(prevForm => {
      const newConfig = JSON.parse(JSON.stringify(prevForm.configuration)); // Deep copy

      if (isChecked) {
        // Add the feature
        if (!newConfig[groupName]) {
          newConfig[groupName] = {};
        }
        newConfig[groupName][feature.name] = {
          type: feature.type,
          value: feature.value, // Start with the camera's current value
        };
      } else {
        // Remove the feature
        if (newConfig[groupName]) {
          delete newConfig[groupName][feature.name];
          if (Object.keys(newConfig[groupName]).length === 0) {
            delete newConfig[groupName];
          }
        }
      }
      return { ...prevForm, configuration: newConfig };
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

  const handleSaveNotes = async () => {
    if (!selectedCamera) return;
    try {
        const response = await fetch(`${ROS_API_URL}/camera/${selectedCamera.identifier}/notes`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ notes: editingNotes }),
        });
        const data = await response.json();
        if (response.ok) {
            setMessage('Notes saved successfully.');
            // This will rely on the refresh interval to update the view
        } else {
            setMessage(`Error saving notes: ${data.detail}`);
        }
    } catch (error) {
        setMessage(`Error saving notes: ${(error as Error).message}`);
    }
  };

  const handleSetPublishingPreset = async (presetName: string) => {
    if (!selectedCamera) return;
    try {
        const response = await fetch(`${ROS_API_URL}/camera/${selectedCamera.identifier}/publishing_preset`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ preset_name: presetName }),
        });
        const data = await response.json();
        if (response.ok) {
            setMessage('Publishing preset set successfully.');
        } else {
            setMessage(`Error setting publishing preset: ${data.detail}`);
        }
    } catch (error) {
        setMessage(`Error setting publishing preset: ${(error as Error).message}`);
    }
  };

  return (isClient&&<div className="w-full h-full text-black bg-gray-100 p-8">
      <Sidebar isOpen={isSettingsOpen} onClose={() => setIsSettingsOpen(false)}>
        <Settings />
      </Sidebar>
      <button
        onClick={() => dispatch(setIsSettingsOpen(true))}
        className="fixed top-4 right-4 bg-blue-600 text-white p-2 rounded-md shadow-lg z-30"
      >
        Settings
      </button>
      <div className="flex justify-center items-center mb-8">
        <h1 className="text-4xl font-bold text-center">Dashboard</h1>
      </div>
      {/* Removed window size display */}

      {message && showMessage && (
        <div className="bg-blue-100 border-l-4 border-blue-500 text-blue-700 p-4 mb-6 flex justify-between items-center" role="alert">
          <div>
            <p className="font-bold">Info</p>
            <p>{message}</p>
          </div>
          <button
            className="ml-4 text-blue-700 hover:text-blue-900 font-bold text-xl"
            onClick={() => dispatch(setShowMessage(false))}
            aria-label="Close alert"
          >
            &times;
          </button>
        </div>
      )}

      {/* Device Selection and Control Section */}
      <div className="w-full bg-white p-6 rounded-lg shadow-md mb-8 h-[50%] overflow-scroll">
        <DeviceSelectorPanel
  
        />
      </div>

      {selectedCamera ? (
        <div className="flex flex-col md:flex-row gap-4 w-full h-full overflow-scroll " style={{ height: height * 0.5 }}>
          <DeviceDetailsPanel
            selectedCamera={selectedCamera}
            editingNotes={editingNotes}
            setEditingNotes={setEditingNotes}
            handleSaveNotes={handleSaveNotes}
            presets={presets}
            handleSetPublishingPreset={handleSetPublishingPreset}
          />
          {/* Right: Settings Panel with Tabs */}
          <div className="w-full md:w-[80%] bg-gray-50 p-4 rounded-md">
            <div className="flex overflow-x-auto space-x-2 mb-4 pb-2 border-b">
              <button
                onClick={() => setActiveTab('features')}
                className={`px-4 py-2 rounded-md text-sm font-medium whitespace-nowrap ${activeTab === 'features' ? 'bg-blue-600 text-white' : 'bg-gray-200 hover:bg-gray-300'
                  }`}
              >
                Features
              </button>
              <button
                onClick={() => {
                  setActiveTab('presets');
                  // Fetch presets when switching to the presets tab
                  if (selectedCamera) {
                    actionsRef.current.fetchPresets(selectedCamera.identifier);
                  }
                }}
                className={`px-4 py-2 rounded-md text-sm font-medium whitespace-nowrap ${activeTab === 'presets' ? 'bg-blue-600 text-white' : 'bg-gray-200 hover:bg-gray-300'
                  }`}
              >
                Presets
              </button>
            </div>

            {/* Render content based on active tab */}
            {activeTab === 'features' ? (
              <FeaturePanel
                selectedCamera={selectedCamera}
                selectedFeatureGroup={selectedFeatureGroup}
                handleFeatureGroupSelect={handleFeatureGroupSelect}
                handleFeatureChange={handleFeatureChange}
              />
            ) : (
              <PresetsPanel
                selectedCamera={selectedCamera}
                presets={presets}
                presetForm={presetForm}
                presetFormError={presetFormError}
                selectedPresetForEditing={selectedPresetForEditing}
                activeTab={activeTab}
                handleCreatePreset={handleCreatePreset}
                handleUpdatePreset={handleUpdatePreset}
                handleDeletePreset={handleDeletePreset}
                loadPresetForEditing={loadPresetForEditing}
                handleCancelPresetEdit={handleCancelPresetEdit}
                handlePresetValueChange={handlePresetValueChange}
                handleTogglePresetFeature={handleTogglePresetFeature}
                dispatch={dispatch}
                setPresetForm={setPresetForm}
                setSelectedPresetForEditing={setSelectedPresetForEditing}
                setActiveTab={setActiveTab}
              />
            )}
          </div>
        </div>
      ) : (
        <p className="text-center text-black">Select a device to view its data and settings.</p>
      )}


      {/* Web Terminal Section */}
      <div className="w-full bg-white p-6 rounded-lg shadow-md mb-8">
        <h2 className="text-2xl font-semibold mb-4">Web Terminal</h2>
        <WebTerminal websocketUrl={TERMINAL_WEBSOCKET_URL} />
      </div>

      {/* Webviz Section */}
      <div className="w-full bg-white p-6 rounded-lg shadow-md mb-8">
        <h2 className="text-2xl font-semibold mb-4">Webviz</h2>
        <div className="w-full h-96 border rounded-md">
          <iframe src="/webviz" className="w-full h-full" />
        </div>
      </div>
      </div>);
}

export default App;
