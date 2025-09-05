"use client";

import { useState, useEffect } from 'react';
import yaml from 'js-yaml'; // Import js-yaml

// Define interfaces for better type safety
interface Feature {
  name: string;
  type: string;
  value: any;
  tooltip?: string;
  description?: string;
  min?: number | string;
  max?: number | string;
  options?: string[];
  representation?: string;
}

interface FeatureGroup {
  name: string;
  source: string; // "group" or "category"
  features: Feature[];
}

interface CameraData {
  id: number;
  identifier: string;
  type: string;
  config: any;
  status: string;
  name: string;
  path: string;
  apiEndpoint: string;
  fileName: string;
  features: FeatureGroup[];
}

export default function ImagingSourcePage() {
  // Driver Control State
  const [isRunning, setIsRunning] = useState(false);
  const [driverLoading, setDriverLoading] = useState(true);
  const [driverMessage, setDriverMessage] = useState('');

  // Configuration State
  const [config, setConfig] = useState<any>({});
  const [configLoading, setConfigLoading] = useState(true);
  const [configMessage, setConfigMessage] = useState('');
  const [isEditing, setIsEditing] = useState(false);
  const [editedConfig, setEditedConfig] = useState<string>('');

  // Camera Features State
  const [cameraData, setCameraData] = useState<CameraData | null>(null);
  const [cameraDataLoading, setCameraDataLoading] = useState(true);
  const [cameraDataMessage, setCameraDataMessage] = useState('');

  const API_BASE_URL = process.env.NEXT_PUBLIC_API_BASE_URL || "http://localhost:8000";

  useEffect(() => {
    fetchStatus();
    fetchConfig();
    fetchCameraFeatures(); // Fetch camera features on mount
  }, []);

  // Driver Control Functions
  const fetchStatus = async () => {
    setDriverLoading(true);
    try {
      const response = await fetch(`${API_BASE_URL}/driver/camera/status`);
      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
      const data = await response.json();
      setIsRunning(data.status === "running");
    } catch (error: any) {
      console.error("Error fetching ImagingSource driver status:", error);
      setDriverMessage(`Failed to fetch ImagingSource driver status: ${error.message}`);
    } finally {
      setDriverLoading(false);
    }
  };

  const startDriver = async () => {
    setDriverLoading(true);
    setDriverMessage('');
    try {
      const response = await fetch(`${API_BASE_URL}/driver/camera/start`, { method: 'POST' });
      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
      const data = await response.json();
      setDriverMessage(data.message);
      if (data.status === "success") {
        setIsRunning(true);
      }
    } catch (error: any) {
      console.error("Error starting ImagingSource driver:", error);
      setDriverMessage(`Failed to start ImagingSource driver: ${error.message}`);
    } finally {
      setDriverLoading(false);
    }
  };

  const stopDriver = async () => {
    setDriverLoading(true);
    setDriverMessage('');
    try {
      const response = await fetch(`${API_BASE_URL}/driver/camera/stop`, { method: 'POST' });
      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
      const data = await response.json();
      setDriverMessage(data.message);
      if (data.status === "success") {
        setIsRunning(false);
      }
    } catch (error: any) {
      console.error("Error stopping ImagingSource driver:", error);
      setDriverMessage(`Failed to stop ImagingSource driver: ${error.message}`);
    } finally {
      setDriverLoading(false);
    }
  };

  // Configuration Functions
  const fetchConfig = async () => {
    setConfigLoading(true);
    setConfigMessage('');
    try {
      const response = await fetch(`${API_BASE_URL}/config/imaging_source`);
      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
      const data = await response.json();
      setConfig(data);
      setEditedConfig(yaml.dump(data)); // Convert JSON to YAML string for editing
    } catch (error: any) {
      console.error("Error fetching ImagingSource config:", error);
      setConfigMessage(`Failed to fetch ImagingSource configuration: ${error.message}`);
    } finally {
      setConfigLoading(false);
    }
  };

  const handleEditChange = (event: React.ChangeEvent<HTMLTextAreaElement>) => {
    setEditedConfig(event.target.value);
  };

  const saveConfig = async () => {
    setConfigLoading(true);
    setConfigMessage('');
    try {
      const jsonConfig = yaml.load(editedConfig); // Convert YAML string back to JSON
      const response = await fetch(`${API_BASE_URL}/config/imaging_source`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(jsonConfig),
      });
      if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
      const data = await response.json();
      setConfigMessage(data.message);
      if (data.status === "success") {
        setConfig(jsonConfig);
        setIsEditing(false);
      }
    } catch (error: any) {
      console.error("Error saving ImagingSource config:", error);
      setConfigMessage(`Failed to save ImagingSource configuration. Please check YAML syntax: ${error.message}`);
    } finally {
      setConfigLoading(false);
    }
  };

  // Camera Features Fetching
  const fetchCameraFeatures = async () => {
    setCameraDataLoading(true);
    setCameraDataMessage('');
    try {
      // Fetch all cameras and display features of the first one for simplicity.
      // A real application might allow selecting a specific camera.
      const camerasResponse = await fetch(`${API_BASE_URL}/get_all_cams/gigE`);
      if (!camerasResponse.ok) throw new Error(`HTTP error! status: ${camerasResponse.status}`);
      const cameras: CameraData[] = await camerasResponse.json();

      if (cameras && cameras.length > 0) {
        // Assuming we want to display features for the first camera found
        setCameraData(cameras[0]);
      } else {
        setCameraData(null);
        setCameraDataMessage("No cameras found or features could not be loaded.");
      }
    } catch (error: any) {
      console.error("Error fetching camera data:", error);
      setCameraDataMessage(`Failed to fetch camera data: ${error.message}`);
      setCameraData(null);
    } finally {
      setCameraDataLoading(false);
    }
  };

  return (
    <div className="h-full w-full flex flex-col items-center p-4 space-y-8">
      {/* Driver Control Section */}
      <div className="w-full max-w-4xl bg-gray-800 p-6 rounded-lg shadow-lg">
        <h2 className="text-2xl font-semibold mb-4 text-white">ImagingSource Driver Control</h2>
        {driverLoading ? (
          <p className="text-gray-300">Loading driver status...</p>
        ) : (
          <>
            <p className="mb-4 text-gray-200">Status: <span className={`font-bold ${isRunning ? 'text-green-500' : 'text-red-500'}`}>
              {isRunning ? "Running" : "Stopped"}
            </span></p>
            <div className="flex space-x-4">
              <button
                onClick={startDriver}
                disabled={isRunning || driverLoading}
                className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                  isRunning || driverLoading ? 'bg-gray-600 cursor-not-allowed' : 'bg-green-600 hover:bg-green-700'
                }`}
              >
                Start Driver
              </button>
              <button
                onClick={stopDriver}
                disabled={!isRunning || driverLoading}
                className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                  !isRunning || driverLoading ? 'bg-gray-600 cursor-not-allowed' : 'bg-red-600 hover:bg-red-700'
                }`}
              >
                Stop Driver
              </button>
            </div>
            {driverMessage && <p className="mt-4 text-sm text-gray-400">{driverMessage}</p>}
          </>
        )}
      </div>

      {/* Configuration Section */}
      <div className="w-full max-w-4xl bg-gray-800 p-6 rounded-lg shadow-lg">
        <h2 className="text-2xl font-semibold mb-4 text-white">ImagingSource Configuration</h2>
        {configLoading ? (
          <p className="text-gray-300">Loading configuration...</p>
        ) : (
          <>
            {isEditing ? (
              <div className="w-full">
                <textarea
                  className="w-full h-64 p-2 border rounded-md bg-gray-700 text-white font-mono"
                  value={editedConfig}
                  onChange={handleEditChange}
                />
                <div className="flex space-x-4 mt-4">
                  <button
                    onClick={saveConfig}
                    disabled={configLoading}
                    className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                      configLoading ? 'bg-gray-600 cursor-not-allowed' : 'bg-blue-600 hover:bg-blue-700'
                    }`}
                  >
                    Save Configuration
                  </button>
                  <button
                    onClick={() => {
                      setIsEditing(false);
                      setEditedConfig(yaml.dump(config)); // Revert changes
                    }}
                    disabled={configLoading}
                    className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                      configLoading ? 'bg-gray-600 cursor-not-allowed' : 'bg-gray-600 hover:bg-gray-700'
                    }`}
                  >
                    Cancel
                  </button>
                </div>
              </div>
            ) : (
              <div className="w-full">
                <pre className="bg-gray-700 text-white p-4 rounded-md overflow-auto max-h-96">
                  {yaml.dump(config)}
                </pre>
                <button
                  onClick={() => setIsEditing(true)}
                  disabled={configLoading}
                  className={`mt-4 px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                    configLoading ? 'bg-gray-600 cursor-not-allowed' : 'bg-blue-600 hover:bg-blue-700'
                  }`}
                >
                  Edit Configuration
                </button>
              </div>
            )}
            {configMessage && <p className="mt-4 text-sm text-gray-400">{configMessage}</p>}
          </>
        )}
      </div>

      {/* Camera Features Section */}
      <div className="w-full max-w-4xl bg-gray-800 p-6 rounded-lg shadow-lg">
        <h2 className="text-2xl font-semibold mb-4 text-white">Camera Features</h2>
        {cameraDataLoading ? (
          <p className="text-gray-300">Loading camera features...</p>
        ) : cameraData ? (
          cameraData.features && cameraData.features.length > 0 ? (
            cameraData.features.map((group: FeatureGroup) => (
              <div key={group.name} className="mb-6 p-4 border border-gray-700 rounded-md bg-gray-700">
                <h3 className="text-xl font-semibold text-white mb-2 flex justify-between items-center">
                  {group.name}
                  <span className="text-sm font-normal text-gray-400">({group.source})</span>
                </h3>
                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                  {group.features.map((feature: Feature) => (
                    <div key={feature.name} className="bg-gray-600 p-3 rounded-md text-gray-300">
                      <p className="font-medium text-white">{feature.name} ({feature.type})</p>
                      <p>Value: {feature.value !== null && feature.value !== undefined ? feature.value : 'N/A'}</p>
                      {feature.tooltip && <p className="text-xs text-gray-400 mt-1">Tooltip: {feature.tooltip}</p>}
                      {feature.options && feature.options.length > 0 && (
                        <p>Options: {feature.options.join(', ')}</p>
                      )}
                      {/* Add more feature details as needed, e.g., min/max */}
                      {feature.min !== undefined && feature.min !== null && (
                        <p>Min: {feature.min}</p>
                      )}
                      {feature.max !== undefined && feature.max !== null && (
                        <p>Max: {feature.max}</p>
                      )}
                    </div>
                  ))}
                </div>
              </div>
            ))
          ) : (
            <p className="text-gray-400">No features found for this camera.</p>
          )
        ) : (
          <p className="text-gray-400">No camera data available. Please ensure a camera is connected and configured.</p>
        )}
        {cameraDataMessage && <p className="mt-4 text-sm text-gray-400">{cameraDataMessage}</p>}
      </div>
    </div>
  );
}
