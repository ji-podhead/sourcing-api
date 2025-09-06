import { createAsyncThunk } from '@reduxjs/toolkit';
import { setDynamicCameras, setSelectedCamera, setOusterStatus } from './devicesSlice';
import {
  setMessage,
  setShowMessage,
  setPresets,
  setRecordings,
  setSelectedFeatureGroup,
} from './dashboardSlice';
import { setCameraFeatures, clearCameraFeatures } from './cameraSlice';
import { addLogMessage, setLogs, clearLogs } from './logSlice';
  import { DeviceStatus, Preset } from '../types';
import { RootState } from './store';
import yaml from 'js-yaml';
import { ROS_API_URL, LOGS_WEBSOCKET_URL } from '../utils/apiConfig';

export const fetchCameras = createAsyncThunk(
  'devices/fetchCameras',
  async (_, { dispatch, getState }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/get_all_cams/gigE`);
      if (!response.ok) {
        throw new Error(`Error fetching cameras: ${response.statusText}`);
      }
      const data = await response.json();
      const cameras = data || [];
      dispatch(setDynamicCameras(cameras));

      const { devices } = getState() as RootState;
      const { selectedCamera } = devices;

      if (selectedCamera) {
        const updatedSelectedCamera = cameras.find((cam: DeviceStatus) => cam.id === selectedCamera.id);
        dispatch(setSelectedCamera(updatedSelectedCamera || null));
      }
      return cameras;
    } catch (error: unknown) {
      const errorMessage = `Error fetching cameras: ${(error as Error).message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
      throw error;
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

    logsWebSocket.onopen = () => {
      dispatch(addLogMessage("WebSocket connected to log stream"));
    };

    logsWebSocket.onmessage = (event: MessageEvent) => {
      try {
        const message = JSON.parse(event.data);
        if (message.log) {
          dispatch(addLogMessage(message.log));
        }
      } catch (error: unknown) {
        dispatch(addLogMessage(`Error parsing WebSocket message: ${event.data}`));
      }
    };

    logsWebSocket.onerror = (error: Event) => {
      dispatch(addLogMessage(`WebSocket error: ${error}`));
    };

    logsWebSocket.onclose = () => {
      dispatch(addLogMessage("WebSocket disconnected from log stream"));
      logsWebSocket = null;
    };
  }
);

export const applyPreset = createAsyncThunk(
  'presets/applyPreset',
  async ({ deviceIdentifier, presetName }: { deviceIdentifier: string, presetName: string }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${deviceIdentifier}/${presetName}/apply`, {
        method: "POST",
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(`Preset '${presetName}' applied successfully!`));
        dispatch(setShowMessage(true));
        // After applying the preset, fetch the camera details to update the UI
        dispatch(fetchCameraDetails({ cameraId: deviceIdentifier, protocol: 'gigE' })); // Assuming gigE, might need to be dynamic
      } else {
        throw new Error(data.detail || response.statusText);
      }
    } catch (error: unknown) {
      dispatch(setMessage(`Error applying preset: ${(error as Error).message}`));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleCreateCamera = createAsyncThunk(
  'devices/handleCreateCamera',
  async ({ newCameraId, newCameraProtocol }: { newCameraId: string, newCameraProtocol: string }, { dispatch }) => {
    try {
      if (!newCameraId.trim()) {
        throw new Error("Camera ID (IP) cannot be empty.");
      }
      if (newCameraProtocol === 'gigE') {
        const ipRegex = /^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
        if (!ipRegex.test(newCameraId)) {
          throw new Error("Invalid IP address format for GigE camera.");
        }
      }
      const response = await fetch(`${ROS_API_URL}/create_camera`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          id: newCameraId,
          protocol: newCameraProtocol,
        }),
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(`Camera ${newCameraId} created successfully!`));
        dispatch(setShowMessage(true));
        dispatch(fetchCameras());
      } else {
        throw new Error(data.detail || response.statusText);
      }
    } catch (error: unknown) {
      const errorMessage = `Error creating camera: ${(error as Error).message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
      throw error;
    }
  }
);

export const handleAction = createAsyncThunk(
  'devices/handleAction',
  async ({ endpoint, successMessage, errorMessage, device }: { endpoint: string, successMessage: string, errorMessage: string, device: DeviceStatus }, { dispatch }) => {
    try {
      let requestBody: any = {};
      let headers: HeadersInit = { "Content-Type": "application/json" };

      if (device.type === "gigE") {
        requestBody = {
          protocol: device.type,
          camera_id: device.id,
        };
      }

      const response = await fetch(`${ROS_API_URL}${endpoint}`, {
        method: "POST",
        headers: headers,
        body: device.type === "gigE" ? JSON.stringify(requestBody) : undefined,
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(successMessage + (data.message ? ` ${data.message}` : "")));
        dispatch(fetchAllStatuses());
        dispatch(fetchRecordings());
      } else {
        throw new Error(data.detail || response.statusText);
      }
    } catch (error: unknown) {
      const errorMessageText = `${errorMessage}: ${(error as Error).message}`;
      dispatch(setMessage(errorMessageText));
      dispatch(setShowMessage(true));
      throw new Error(errorMessageText);
    }
  }
);

export const handleCreatePreset = createAsyncThunk(
  'presets/handleCreatePreset',
  async ({ deviceIdentifier, presetForm }: { deviceIdentifier: string, presetForm: { name: string, configuration: any } }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${deviceIdentifier}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: presetForm.name, configuration: presetForm.configuration }),
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(`Preset '${presetForm.name}' created successfully!`));
        dispatch(setShowMessage(true));
        dispatch(fetchPresets(deviceIdentifier));
      } else {
        throw new Error(data.detail || response.statusText);
      }
    } catch (error: unknown) {
      dispatch(setMessage(`Error creating preset: ${(error as Error).message}`));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleUpdatePreset = createAsyncThunk(
  'presets/handleUpdatePreset',
  async ({ deviceIdentifier, presetName, presetForm }: { deviceIdentifier: string, presetName: string, presetForm: { name: string, configuration: any } }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${deviceIdentifier}/${presetName}`, {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: presetForm.name, configuration: presetForm.configuration }),
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(`Preset '${presetForm.name}' updated successfully!`));
        dispatch(setShowMessage(true));
        dispatch(fetchPresets(deviceIdentifier));
      } else {
        throw new Error(data.detail || response.statusText);
      }
    } catch (error: unknown) {
      dispatch(setMessage(`Error updating preset: ${(error as Error).message}`));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleDeletePreset = createAsyncThunk(
  'presets/handleDeletePreset',
  async ({ deviceIdentifier, presetName }: { deviceIdentifier: string, presetName: string }, { dispatch }) => {
    if (!window.confirm(`Are you sure you want to delete preset "${presetName}"?`)) {
      return;
    }
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${deviceIdentifier}/${presetName}`, {
        method: "DELETE",
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage(`Preset '${presetName}' deleted successfully!`));
        dispatch(setShowMessage(true));
        dispatch(fetchPresets(deviceIdentifier));
      } else {
        throw new Error(data.detail || response.statusText);
      }
    } catch (error: unknown) {
      dispatch(setMessage(`Error deleting preset: ${(error as Error).message}`));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleSaveNotes = createAsyncThunk(
  'camera/handleSaveNotes',
  async ({ cameraIdentifier, notes }: { cameraIdentifier: string, notes: string }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/camera/${cameraIdentifier}/notes`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ notes }),
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage('Notes saved successfully.'));
      } else {
        throw new Error(data.detail);
      }
    } catch (error) {
      dispatch(setMessage(`Error saving notes: ${(error as Error).message}`));
    }
  }
);

export const handleSetPublishingPreset = createAsyncThunk(
  'camera/handleSetPublishingPreset',
  async ({ cameraIdentifier, presetName }: { cameraIdentifier: string, presetName: string }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/camera/${cameraIdentifier}/publishing_preset`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ preset_name: presetName }),
      });
      const data = await response.json();
      if (response.ok) {
        dispatch(setMessage('Publishing preset set successfully.'));
      } else {
        throw new Error(data.detail);
      }
    } catch (error) {
      dispatch(setMessage(`Error setting publishing preset: ${(error as Error).message}`));
    }
  }
);

export const fetchCameraDetails = createAsyncThunk(
  'devices/fetchCameraDetails',
  async ({ cameraId, protocol }: { cameraId: string, protocol: string }, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/get_cam/${protocol}/${cameraId}`);
      if (!response.ok) {
        throw new Error(`Error fetching camera details for ${cameraId}: ${response.statusText}`);
      }
      const data = await response.json();
      const augmentedCamera: DeviceStatus = {
        ...data,
        name: `Camera: ${data.identifier}`,
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
    } catch (error: unknown) {
      const errorMessage = `Error fetching camera details for ${cameraId}: ${(error as Error).message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
      dispatch(setSelectedCamera(null));
      dispatch(clearCameraFeatures());
      throw error;
    }
  }
);

export const fetchPresets = createAsyncThunk(
  'dashboard/fetchPresets',
  async (deviceIdentifier: string, { dispatch }) => {
    try {
      const response = await fetch(`${ROS_API_URL}/presets/${deviceIdentifier}`);
      if (!response.ok) {
        throw new Error(`Error fetching presets for device ${deviceIdentifier}: ${response.statusText}`);
      }
      const data = await response.json();
      dispatch(setPresets(data.presets || []));
      return data.presets || [];
    } catch (error: unknown) {
      const errorMessage = `Error fetching presets for device ${deviceIdentifier}: ${(error as Error).message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
      dispatch(setPresets([]));
      throw error;
    }
  }
);

export const fetchAllStatuses = createAsyncThunk(
  'devices/fetchAllStatuses',
  async (_, { dispatch, getState }) => {
    const { devices } = getState() as RootState;
    const { dynamicCameras, selectedCamera } = devices;

    const fetchStatus = async (endpoint: string, protocol?: string, cameraId?: string): Promise<string> => {
      try {
        let url = `${ROS_API_URL}${endpoint}`;
        if (protocol && cameraId) {
          url += `?protocol=${protocol}&camera_id=${cameraId}`;
        }
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

    const ousterStatusResult = await fetchStatus("/driver/ouster/status");
    dispatch(setOusterStatus(ousterStatusResult));

    const updatedDynamicCameras = await Promise.all(
      dynamicCameras.map(async (cam) => {
        const status = await fetchStatus(
          `/driver/${cam.apiEndpoint.split('/').pop()}/status`,
          cam.type,
          cam.id
        );
        return { ...cam, status };
      })
    );

    dispatch(setDynamicCameras(updatedDynamicCameras));

    if (selectedCamera) {
      const updatedSelected = updatedDynamicCameras.find((c) => c.id === selectedCamera.id);
      if (updatedSelected && updatedSelected.status !== selectedCamera.status) {
        dispatch(setSelectedCamera(updatedSelected));
      }
      // Also fetch the details for the selected camera to update its features
      dispatch(fetchCameraDetails({ cameraId: selectedCamera.id, protocol: selectedCamera.type }));
    }
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
    } catch (error: unknown) {
      const errorMessage = `Error fetching recordings: ${(error as Error).message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
    }
  }
);

export const handleFeatureChange = createAsyncThunk(
  'camera/handleFeatureChange',
  async ({ cameraId, featureName, newValue, protocol }: { cameraId: string, featureName: string, newValue: any, protocol: string }, { dispatch }) => {
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
        dispatch(fetchCameraDetails({ cameraId, protocol }));
      } else {
        throw new Error(data.detail || `Failed to set feature ${featureName}`);
      }
    } catch (error: unknown) {
      const errorMessage = `Error updating feature ${featureName}: ${(error as Error).message}`;
      dispatch(setMessage(errorMessage));
      dispatch(setShowMessage(true));
    }
  }
);
