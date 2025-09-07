import { createAsyncThunk } from '@reduxjs/toolkit';
import { setDynamicCameras, setSelectedCamera, setOusterStatus } from './devicesSlice';
import {
  setMessage,
  setShowMessage,
  setPresets,
  setRecordings,
  setSelectedFeatureGroup,
  setDiscoveredCameras,
  setIsDiscoveryPanelOpen,
} from './dashboardSlice';
import { setCameraFeatures, clearCameraFeatures } from './cameraSlice';
import { addLogMessage } from './logSlice';
import { DeviceStatus, Preset } from '../types';
import { RootState } from './store';
import { ROS_API_URL, LOGS_WEBSOCKET_URL } from '../utils/apiConfig';

export const fetchCameras = createAsyncThunk(
  'devices/fetchCameras',
  async (_, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/cameras`);
      if (!response.ok) throw new Error(`Error fetching cameras: ${response.statusText}`);
      const data = await response.json();
      dispatch(setDynamicCameras(data || []));
      return data || [];
    } catch (error: any) {
      const errorMessage = `Error fetching cameras: ${error.message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
      throw error;
    }
  }
);

export const discoverCameras = createAsyncThunk(
    'devices/discoverCameras',
    async (_, { dispatch }) => {
        try {
            const response = await fetch(`${ROS_API_URL}/discover_cameras`);
            if (!response.ok) throw new Error('Failed to discover cameras');
            const data = await response.json();
            dispatch(setDiscoveredCameras(data));
        } catch (error: any) {
            dispatch(setMessage(`Error discovering cameras: ${error.message}`));
            dispatch(setShowMessage(true));
        }
    }
);

let logsWebSocket: WebSocket | null = null;

export const connectToLogs = createAsyncThunk(
  'logs/connectToLogs',
  async (_, { dispatch }) => {
    if (logsWebSocket) {
      logsWebSocket.close();
    }
    logsWebSocket = new WebSocket(LOGS_WEBSOCKET_URL);
    logsWebSocket.onopen = () => dispatch(addLogMessage("WebSocket connected to log stream"));
    logsWebSocket.onmessage = (event: MessageEvent) => {
      try {
        const message = JSON.parse(event.data);
        if (message.log) dispatch(addLogMessage(message.log));
      } catch (error) {
        dispatch(addLogMessage(`Error parsing WebSocket message: ${event.data}`));
      }
    };
    logsWebSocket.onerror = (error: Event) => dispatch(addLogMessage(`WebSocket error: ${error}`));
    logsWebSocket.onclose = () => {
      dispatch(addLogMessage("WebSocket disconnected from log stream"));
      logsWebSocket = null;
    };
  }
);

export const applyPreset = createAsyncThunk(
  'presets/applyPreset',
  async ({ cameraId, presetName }: { cameraId: number, presetName: string }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/publishing_preset`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ preset_name: presetName }),
      });
      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || response.statusText);
      }
      dispatch(setMessage(`Preset '${presetName}' applied successfully!`));
      dispatch(setShowMessage(true));
      dispatch(fetchCameraDetails(cameraId));
    } catch (error: any) {
      dispatch(setMessage(`Error applying preset: ${error.message}`));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleCreateCamera = createAsyncThunk(
  'devices/handleCreateCamera',
  async ({ identifier }: { identifier: string }, { dispatch }) => {
    try {
      if (!identifier.trim()) throw new Error("Identifier cannot be empty.");

      const response = await fetch(`${ROS_API_URL}/camera`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ identifier }),
      });
      const newCamera = await response.json();
      if (!response.ok) throw new Error(newCamera.detail || response.statusText);

      dispatch(setMessage(`Camera created successfully with ID: ${newCamera.id}!`));
      dispatch(setShowMessage(true));
      await dispatch(fetchCameras());
      dispatch(setIsDiscoveryPanelOpen(false));
    } catch (error: any) {
      const errorMessage = `Error creating camera: ${error.message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
      throw error;
    }
  }
);

export const handleAction = createAsyncThunk(
  'devices/handleAction',
  async ({ endpoint, successMessage, errorMessage }: { endpoint: string, successMessage: string, errorMessage: string }, { dispatch }) => {
    try {
      const url = `${ROS_API_URL}${endpoint}`;
      const response = await fetch(url, { method: "POST" });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(successMessage + (data.message ? ` ${data.message}` : "")));
        dispatch(fetchAllStatuses());
      } else {
        throw new Error(data.detail || response.statusText);
      }
    } catch (error: any) {
      const errorMessageText = `${errorMessage}: ${error.message}`;
      dispatch(setMessage(errorMessageText));
      dispatch(setShowMessage(true));
      throw new Error(errorMessageText);
    }
  }
);

export const handleCreatePreset = createAsyncThunk(
  'presets/handleCreatePreset',
  async ({ cameraId, presetForm }: { cameraId: number, presetForm: { name: string, configuration: any } }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${cameraId}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: presetForm.name, configuration: presetForm.configuration }),
      });
      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || response.statusText);
      }
      dispatch(setMessage(`Preset '${presetForm.name}' created successfully!`));
      dispatch(setShowMessage(true));
      dispatch(fetchPresets(cameraId));
    } catch (error: any) {
      dispatch(setMessage(`Error creating preset: ${error.message}`));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleUpdatePreset = createAsyncThunk(
  'presets/handleUpdatePreset',
  async ({ cameraId, presetName, presetForm }: { cameraId: number, presetName: string, presetForm: { name: string, configuration: any } }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${cameraId}/${presetName}`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: presetForm.name, configuration: presetForm.configuration }),
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(`Preset '${presetForm.name}' updated successfully!`));
        dispatch(setShowMessage(true));
        dispatch(fetchPresets(cameraId));
      } else {
        throw new Error(data.detail || response.statusText);
      }
    } catch (error: any) {
      dispatch(setMessage(`Error updating preset: ${error.message}`));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleDeletePreset = createAsyncThunk(
  'presets/handleDeletePreset',
  async ({ cameraId, presetName }: { cameraId: number, presetName: string }, { dispatch }) => {
    if (!window.confirm(`Are you sure you want to delete preset "${presetName}"?`)) {
      return;
    }
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${cameraId}/${presetName}`, {
        method: "DELETE",
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(`Preset '${presetName}' deleted successfully!`));
        dispatch(setShowMessage(true));
        dispatch(fetchPresets(cameraId));
      } else {
        throw new Error(data.detail || response.statusText);
      }
    } catch (error: any) {
      dispatch(setMessage(`Error deleting preset: ${error.message}`));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleSaveNotes = createAsyncThunk(
  'camera/handleSaveNotes',
  async ({ cameraId, notes }: { cameraId: number, notes: string }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/notes`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ notes }),
      });
      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail);
      }
      dispatch(setMessage('Notes saved successfully.'));
      dispatch(setShowMessage(true));
    } catch (error: any) {
      dispatch(setMessage(`Error saving notes: ${error.message}`));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleSetPublishingPreset = createAsyncThunk(
  'camera/handleSetPublishingPreset',
  async ({ cameraId, presetName }: { cameraId: number, presetName: string }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/publishing_preset`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ preset_name: presetName }),
      });
      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail);
      }
      dispatch(setMessage('Publishing preset set successfully.'));
      dispatch(setShowMessage(true));
      dispatch(fetchCameraDetails(cameraId));
    } catch (error: any) {
      dispatch(setMessage(`Error setting publishing preset: ${error.message}`));
      dispatch(setShowMessage(true));
    }
  }
);

export const fetchCameraDetails = createAsyncThunk(
  'devices/fetchCameraDetails',
  async (cameraId: number, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/camera/${cameraId}`);
      if (!response.ok) throw new Error(`Error fetching camera details for ${cameraId}: ${response.statusText}`);

      const data = await response.json();
      const augmentedCamera: DeviceStatus = {
        ...data,
        name: `Camera: ${data.camera_name}`,
        path: `/imagingsource`,
        apiEndpoint: `/driver/camera`,
        fileName: `camera_${data.id}.yaml`,
      };
      dispatch(setSelectedCamera(augmentedCamera));
      if (augmentedCamera.features && augmentedCamera.features.length > 0) {
        dispatch(setCameraFeatures(augmentedCamera.features));
        dispatch(setSelectedFeatureGroup(augmentedCamera.features[0].name));
      } else {
        dispatch(clearCameraFeatures());
        dispatch(setSelectedFeatureGroup(null));
      }
      return augmentedCamera;
    } catch (error: any) {
      const errorMessage = `Error fetching camera details: ${error.message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
      throw error;
    }
  }
);

export const fetchPresets = createAsyncThunk(
  'dashboard/fetchPresets',
  async (cameraId: number, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${cameraId}`);
      if (!response.ok) throw new Error(`Error fetching presets for device ${cameraId}: ${response.statusText}`);
      const data = await response.json();
      dispatch(setPresets(data.presets || []));
    } catch (error: any) {
      const errorMessage = `Error fetching presets: ${error.message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
    }
  }
);

export const fetchAllStatuses = createAsyncThunk(
  'devices/fetchAllStatuses',
  async (_, { dispatch, getState }) => {
    const { devices } = getState() as RootState;
    const { dynamicCameras } = devices;
    const fetchStatus = async (endpoint: string, cameraId?: number): Promise<string> => {
      try {
        const url = `${ROS_API_URL}${endpoint}/${cameraId}/status`;
        const response = await fetch(url);
        if (response.ok) {
          const data = await response.json();
          return data.status;
        }
        return "unknown";
      } catch (error) {
        return "unknown";
      }
    };
    const updatedDynamicCameras = await Promise.all(
      dynamicCameras.map(async (cam) => {
        const status = await fetchStatus(`/driver/camera`, cam.id);
        return { ...cam, status };
      })
    );
    dispatch(setDynamicCameras(updatedDynamicCameras));
  }
);

export const fetchRecordings = createAsyncThunk(
  'dashboard/fetchRecordings',
  async (_, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/recordings`);
      if (response.ok) {
        const data = await response.json();
        dispatch(setRecordings(Array.isArray(data.recordings) ? data.recordings : []));
      } else {
        throw new Error(`Error fetching recordings: ${response.statusText}`);
      }
    } catch (error: any) {
      const errorMessage = `Error fetching recordings: ${error.message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleFeatureChange = createAsyncThunk(
  'camera/handleFeatureChange',
  async ({ cameraId, featureName, newValue }: { cameraId: number, featureName: string, newValue: any }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/set_feature`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ feature: featureName, value: newValue }),
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(`Feature ${featureName} updated successfully!`));
        dispatch(setShowMessage(true));
        dispatch(fetchCameraDetails(cameraId));
      } else {
        throw new Error(data.detail || `Failed to set feature ${featureName}`);
      }
    } catch (error: any) {
      const errorMessage = `Error updating feature ${featureName}: ${error.message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleDeleteCamera = createAsyncThunk(
  'devices/handleDeleteCamera',
  async (cameraId: number, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/camera/${cameraId}`, {
        method: 'DELETE',
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(data.message));
        dispatch(setShowMessage(true));
        dispatch(fetchCameras());
      } else {
        throw new Error(data.detail || 'Failed to delete camera');
      }
    } catch (error: any) {
      dispatch(setMessage(`Error deleting camera: ${error.message}`));
      dispatch(setShowMessage(true));
    }
  }
);
