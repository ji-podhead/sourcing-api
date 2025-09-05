import { useDispatch } from 'react-redux';
import { RootState } from '../redux/store';
import { DeviceStatus, Preset } from '../types';
import dashboardSliceActions from '../redux/dashboardSlice'; // Assuming default export
import devicesSliceActions from '../redux/devicesSlice'; // Assuming default export

const ROS_API_URL = "http://localhost:8000";

// --- Device and Camera Handlers ---

export const fetchCameras = async (dispatch: any, getState: () => RootState) => {
  try {
    const response = await fetch(`${ROS_API_URL}/get_all_cams/gigE`);
    if (response.ok) {
      const data = await response.json();
      dispatch(dashboardSliceActions.setDynamicCameras(data || []));
      const { devices } = getState();
      if (devices.selectedCamera) {
        const updatedSelectedCamera = data.find((cam: DeviceStatus) => cam.id === devices.selectedCamera.id);
        if (updatedSelectedCamera) {
          dispatch(devicesSliceActions.setSelectedCamera(updatedSelectedCamera));
        } else {
          dispatch(devicesSliceActions.setSelectedCamera(null)); // Camera no longer exists
        }
      }
    } else {
      dispatch(dashboardSliceActions.setMessage(`Error fetching cameras: ${response.statusText}`));
      dispatch(dashboardSliceActions.setShowMessage(true));
    }
  } catch (error: unknown) {
    dispatch(dashboardSliceActions.setMessage(`Error fetching cameras: ${(error as Error).message}`));
    dispatch(dashboardSliceActions.setShowMessage(true));
  }
};

export const fetchCameraDetails = async (cameraId: string, protocol: string, dispatch: any, getState: () => RootState) => {
  try {
    const response = await fetch(`${ROS_API_URL}/get_cam/${protocol}/${cameraId}`);
    if (response.ok) {
      const data = await response.json();
      console.log("Fetched camera details:", data);
      const augmentedCamera: DeviceStatus = {
        ...data,
        name: `Camera: ${data.identifier}`,
        path: `/imagingsource`,
        apiEndpoint: `/driver/camera`,
        fileName: `camera_${data.id}.yaml`,
      };
      dispatch(devicesSliceActions.setSelectedCamera(augmentedCamera));
      if (augmentedCamera.features && augmentedCamera.features.length > 0) {
        dispatch(dashboardSliceActions.setSelectedFeatureGroup(augmentedCamera.features[0].name));
      } else {
        dispatch(dashboardSliceActions.setSelectedFeatureGroup(null));
      }
    } else {
      dispatch(dashboardSliceActions.setMessage(`Error fetching camera details for ${cameraId}: ${response.statusText}`));
      dispatch(dashboardSliceActions.setShowMessage(true));
      dispatch(devicesSliceActions.setSelectedCamera(null));
    }
  } catch (error: unknown) {
    dispatch(dashboardSliceActions.setMessage(`Error fetching camera details for ${cameraId}: ${(error as Error).message}`));
    dispatch(dashboardSliceActions.setShowMessage(true));
    dispatch(devicesSliceActions.setSelectedCamera(null));
  }
};

export const fetchPresets = async (deviceIdentifier: string, dispatch: any, getState: () => RootState) => {
  try {
    const response = await fetch(`${ROS_API_URL}/presets/${deviceIdentifier}`);
    if (response.ok) {
      const data = await response.json();
      dispatch(dashboardSliceActions.setPresets(data.presets || []));
    } else {
      dispatch(dashboardSliceActions.setMessage(`Error fetching presets for device ${deviceIdentifier}: ${response.statusText}`));
      dispatch(dashboardSliceActions.setShowMessage(true));
      dispatch(dashboardSliceActions.setPresets([]));
    }
  } catch (error: unknown) {
    dispatch(dashboardSliceActions.setMessage(`Error fetching presets for device ${deviceIdentifier}: ${(error as Error).message}`));
    dispatch(dashboardSliceActions.setShowMessage(true));
    dispatch(dashboardSliceActions.setPresets([]));
  }
};

export const fetchRecordings = async (dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/recordings`);
    if (response.ok) {
      const data = await response.json();
      if (!Array.isArray(data.recordings)) {
        data.recordings = [];
      }
      dispatch(dashboardSliceActions.setRecordings(data.recordings));
    } else {
      dispatch(dashboardSliceActions.setMessage(`Error fetching recordings: ${response.statusText}`));
      dispatch(dashboardSliceActions.setShowMessage(true));
    }
  } catch (error: unknown) {
    dispatch(dashboardSliceActions.setMessage(`Error fetching recordings: ${(error as Error).message}`));
    dispatch(dashboardSliceActions.setShowMessage(true));
  }
};

export const fetchDeviceConfig = async (deviceInfo: { name: string; fileName: string }, dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/config/${deviceInfo.fileName}`);
    if (response.ok) {
      const data = await response.json();
      // Assuming yaml.dump is available globally or imported elsewhere if needed
      // For now, let's assume it's available or we'll need to import it.
      // If yaml is not available, this part needs adjustment.
      // For now, let's just log the data.
      console.log(`Config for ${deviceInfo.name}:`, data);
      // dispatch(setDeviceConfigs(prevConfigs => ({ ...prevConfigs, [deviceInfo.name]: yaml.dump(data, { indent: 2 }) })));
    } else {
      console.error(`Error fetching ${deviceInfo.fileName}: ${response.statusText}`);
    }
  } catch (error: unknown) {
    console.error(`Error fetching ${deviceInfo.fileName}: ${(error as Error).message}`);
  }
};

export const handleAction = async (
  endpoint: string,
  successMessage: string,
  errorMessage: string,
  device: DeviceStatus,
  dispatch: any,
  getState: () => RootState
) => {
  dispatch(dashboardSliceActions.setMessage("")); // Clear previous messages
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
      dispatch(dashboardSliceActions.setMessage(successMessage + (data.message ? ` ${data.message}` : "")));

      const { devices } = getState(); // Get current state
      if (device.type === "gigE") {
        dispatch(dashboardSliceActions.setDynamicCameras(devices.dynamicCameras.map((cam: DeviceStatus) =>
          cam.id === device.id
            ? { ...cam, status: data.status === "success" ? "running" : "stopped" }
            : cam
        )));
        if (devices.selectedCamera && devices.selectedCamera.id === device.id) {
          dispatch(devicesSliceActions.setSelectedCamera(prev => prev ? { ...prev, status: data.status === "success" ? "running" : "stopped" } : null));
        }
      } else if (device.name === "Ouster Lidar") {
        dispatch(dashboardSliceActions.setOusterStatus(data.status === "success" ? "running" : "stopped"));
      }
      fetchRecordings(dispatch); // Refresh recordings
    } else {
      dispatch(dashboardSliceActions.setMessage(`${errorMessage}: ${data.detail || response.statusText}`));
    }
  } catch (error: unknown) {
    dispatch(dashboardSliceActions.setMessage(`${errorMessage}: ${(error as Error).message}`));
  }
};

export const handleSaveNotes = async (cameraId: string, notes: string, dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/notes`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ notes }),
    });
    const data = await response.json();
    if (response.ok) {
      dispatch(dashboardSliceActions.setMessage('Notes saved successfully.'));
      dispatch(dashboardSliceActions.setShowMessage(true));
    } else {
      dispatch(dashboardSliceActions.setMessage(`Error saving notes: ${data.detail}`));
      dispatch(dashboardSliceActions.setShowMessage(true));
    }
  } catch (error) {
    dispatch(dashboardSliceActions.setMessage(`Error saving notes: ${(error as Error).message}`));
    dispatch(dashboardSliceActions.setShowMessage(true));
  }
};

export const handleSetPublishingPreset = async (cameraId: string, presetName: string, dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/publishing_preset`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.JSON.stringify({ preset_name: presetName }),
    });
    const data = await response.json();
    if (response.ok) {
      dispatch(dashboardSliceActions.setMessage('Publishing preset set successfully.'));
      dispatch(dashboardSliceActions.setShowMessage(true));
    } else {
      dispatch(dashboardSliceActions.setMessage(`Error setting publishing preset: ${data.detail}`));
      dispatch(dashboardSliceActions.setShowMessage(true));
    }
  } catch (error) {
    dispatch(dashboardSliceActions.setMessage(`Error setting publishing preset: ${(error as Error).message}`));
    dispatch(dashboardSliceActions.setShowMessage(true));
  }
};

// Helper to fetch status, used by fetchAllStatuses
const fetchStatus = async (endpoint: string, protocol?: string, cameraId?: string, dispatch?: any): Promise<string> => {
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
      if (dispatch) {
        dispatch(dashboardSliceActions.setMessage(`Error fetching status from ${endpoint}: ${response.statusText}`));
        dispatch(dashboardSliceActions.setShowMessage(true));
      }
      return "unknown";
    }
  } catch (error: unknown) {
    if (dispatch) {
      dispatch(dashboardSliceActions.setMessage(`Error fetching status from ${endpoint}: ${(error as Error).message}`));
      dispatch(dashboardSliceActions.setShowMessage(true));
    }
    return "unknown";
  }
};

// Function to fetch all statuses (static and dynamic)
export const fetchAllStatuses = async (dispatch: any, getState: () => RootState) => {
  const { devices } = getState();
  const staticDevicesInfo = [
    { name: "Ouster Lidar", path: "/ouster", apiEndpoint: "/driver/ouster", fileName: "ouster.yaml" },
  ]; // Define staticDevicesInfo here or pass it as an argument

  // Fetch status for static devices
  const ousterStatusResult = await fetchStatus("/driver/ouster/status", undefined, undefined, dispatch);
  dispatch(dashboardSliceActions.setOusterStatus(ousterStatusResult));

  // Fetch status for dynamic cameras
  const updatedDynamicCameras = await Promise.all(
    devices.dynamicCameras.map(async (cam: DeviceStatus) => {
      const status = await fetchStatus(
        `/driver/${cam.apiEndpoint.split('/').pop()}/status`,
        cam.type,
        cam.id,
        dispatch
      );
      return { ...cam, status };
    })
  );
  dispatch(dashboardSliceActions.setDynamicCameras(updatedDynamicCameras));

  // If selected camera's status changed, update selectedCamera
  if (devices.selectedCamera) {
    const updatedSelected = updatedDynamicCameras.find((c) => c.id === devices.selectedCamera.id);
    if (updatedSelected && updatedSelected.status !== devices.selectedCamera.status) {
      dispatch(devicesSliceActions.setSelectedCamera(updatedSelected));
    }
  }
};

// Handler for creating a new camera
export const handleCreateCamera = async (newCameraId: string, newCameraProtocol: string, dispatch: any) => {
  dispatch(dashboardSliceActions.setMessage("")); // Clear previous messages
  if (!newCameraId.trim()) {
    dispatch(dashboardSliceActions.setMessage("Camera ID (IP) cannot be empty."));
    dispatch(dashboardSliceActions.setShowMessage(true));
    return;
  }

  if (newCameraProtocol === 'gigE') {
    const ipRegex = /^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
    if (!ipRegex.test(newCameraId)) {
      dispatch(dashboardSliceActions.setMessage("Invalid IP address format for GigE camera."));
      dispatch(dashboardSliceActions.setShowMessage(true));
      return;
    }
  }

  try {
    const response = await fetch(`${ROS_API_URL}/create_camera`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.JSON.stringify({
        id: newCameraId,
        protocol: newCameraProtocol,
      }),
    });
    const data = await response.json();
    if (response.ok) {
      dispatch(dashboardSliceActions.setMessage(`Camera ${newCameraId} created successfully!`));
      dispatch(dashboardSliceActions.setShowMessage(true));
      fetchCameras(dispatch, () => ({ devices: { dynamicCameras: [], selectedCamera: null } })); // Refresh the list of cameras (mock state for dispatch)
    } else {
      dispatch(dashboardSliceActions.setMessage(`Error creating camera: ${data.detail || response.statusText}`));
      dispatch(dashboardSliceActions.setShowMessage(true));
    }
  } catch (error: unknown) {
    dispatch(dashboardSliceActions.setMessage(`Error creating camera: ${(error as Error).message}`));
    dispatch(dashboardSliceActions.setShowMessage(true));
  }
};

// Handler for changing a feature value
export const handleFeatureChange = async (
  cameraId: string,
  featureGroupName: string,
  featureName: string,
  newValue: any,
  dispatch: any,
  getState: () => RootState
) => {
  dispatch(dashboardSliceActions.setMessage(""));
  try {
    // Placeholder for actual API call to update feature
    console.log(`Simulating update for feature ${featureName} in group ${featureGroupName} for camera ${cameraId} to ${newValue}`);

    // Update the local state immediately for better UX
    const { devices } = getState();
    const currentSelectedCamera = devices.selectedCamera;

    if (!currentSelectedCamera) {
      dispatch(dashboardSliceActions.setMessage("No camera selected."));
      dispatch(dashboardSliceActions.setShowMessage(true));
      return;
    }

    const updatedFeatures = currentSelectedCamera.features?.map(group => {
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

    const updatedCamera = { ...currentSelectedCamera, features: updatedFeatures };
    dispatch(devicesSliceActions.setSelectedCamera(updatedCamera));

    // In a real scenario, you would call an API endpoint here:
    // const response = await fetch(`${ROS_API_URL}/update_feature`, { ... });
    // const data = await response.json();
    // if (response.ok) { ... } else { ... }

    // Re-fetch details to sync state from backend
    await fetchCameraDetails(cameraId, currentSelectedCamera.type || 'gigE', dispatch, getState);

    dispatch(dashboardSliceActions.setMessage(`Feature ${featureName} updated successfully!`));
    dispatch(dashboardSliceActions.setShowMessage(true));

  } catch (error: unknown) {
    dispatch(dashboardSliceActions.setMessage(`Error updating feature ${featureName}: ${(error as Error).message}`));
    dispatch(dashboardSliceActions.setShowMessage(true));
  }
};

export const handleSaveNotes = async (cameraId: string, notes: string, dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/notes`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ notes }),
    });
    const data = await response.json();
    if (response.ok) {
      dispatch(dashboardSliceActions.setMessage('Notes saved successfully.'));
      dispatch(dashboardSliceActions.setShowMessage(true));
    } else {
      dispatch(dashboardSliceActions.setMessage(`Error saving notes: ${data.detail}`));
      dispatch(dashboardSliceActions.setShowMessage(true));
    }
  } catch (error) {
    dispatch(dashboardSliceActions.setMessage(`Error saving notes: ${(error as Error).message}`));
    dispatch(dashboardSliceActions.setShowMessage(true));
  }
};

export const handleSetPublishingPreset = async (cameraId: string, presetName: string, dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/publishing_preset`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ preset_name: presetName }),
    });
    const data = await response.json();
    if (response.ok) {
      dispatch(dashboardSliceActions.setMessage('Publishing preset set successfully.'));
      dispatch(dashboardSliceActions.setShowMessage(true));
    } else {
      dispatch(dashboardSliceActions.setMessage(`Error setting publishing preset: ${data.detail}`));
      dispatch(dashboardSliceActions.setShowMessage(true));
    }
  } catch (error) {
    dispatch(dashboardSliceActions.setMessage(`Error setting publishing preset: ${(error as Error).message}`));
    dispatch(dashboardSliceActions.setShowMessage(true));
  }
};

const ROS_API_URL = "http://localhost:8000";

// --- Device and Camera Handlers ---

export const fetchCameras = async (dispatch: any, getState: () => RootState) => {
  try {
    const response = await fetch(`${ROS_API_URL}/get_all_cams/gigE`);
    if (response.ok) {
      const data = await response.json();
      dispatch(setDynamicCameras(data || []));
      const { devices } = getState();
      if (devices.selectedCamera) {
        const updatedSelectedCamera = data.find((cam: DeviceStatus) => cam.id === devices.selectedCamera.id);
        if (updatedSelectedCamera) {
          dispatch(setSelectedCameraAction(updatedSelectedCamera));
        } else {
          dispatch(setSelectedCameraAction(null)); // Camera no longer exists
        }
      }
    } else {
      dispatch(setMessage(`Error fetching cameras: ${response.statusText}`));
      dispatch(setShowMessage(true));
    }
  } catch (error: unknown) {
    dispatch(setMessage(`Error fetching cameras: ${(error as Error).message}`));
    dispatch(setShowMessage(true));
  }
};

export const fetchCameraDetails = async (cameraId: string, protocol: string, dispatch: any, getState: () => RootState) => {
  try {
    const response = await fetch(`${ROS_API_URL}/get_cam/${protocol}/${cameraId}`);
    if (response.ok) {
      const data = await response.json();
      console.log("Fetched camera details:", data);
      const augmentedCamera: DeviceStatus = {
        ...data,
        name: `Camera: ${data.identifier}`,
        path: `/imagingsource`,
        apiEndpoint: `/driver/camera`,
        fileName: `camera_${data.id}.yaml`,
      };
      dispatch(setSelectedCameraAction(augmentedCamera));
      if (augmentedCamera.features && augmentedCamera.features.length > 0) {
        dispatch(setSelectedFeatureGroup(augmentedCamera.features[0].name));
      } else {
        dispatch(setSelectedFeatureGroup(null));
      }
    } else {
      dispatch(setMessage(`Error fetching camera details for ${cameraId}: ${response.statusText}`));
      dispatch(setShowMessage(true));
      dispatch(setSelectedCameraAction(null));
    }
  } catch (error: unknown) {
    dispatch(setMessage(`Error fetching camera details for ${cameraId}: ${(error as Error).message}`));
    dispatch(setShowMessage(true));
    dispatch(setSelectedCameraAction(null));
  }
};

export const fetchPresets = async (deviceIdentifier: string, dispatch: any, getState: () => RootState) => {
  try {
    const response = await fetch(`${ROS_API_URL}/presets/${deviceIdentifier}`);
    if (response.ok) {
      const data = await response.json();
      dispatch(setPresets(data.presets || []));
    } else {
      dispatch(setMessage(`Error fetching presets for device ${deviceIdentifier}: ${response.statusText}`));
      dispatch(setShowMessage(true));
      dispatch(setPresets([]));
    }
  } catch (error: unknown) {
    dispatch(setMessage(`Error fetching presets for device ${deviceIdentifier}: ${(error as Error).message}`));
    dispatch(setShowMessage(true));
    dispatch(setPresets([]));
  }
};

export const fetchRecordings = async (dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/recordings`);
    if (response.ok) {
      const data = await response.json();
      if (!Array.isArray(data.recordings)) {
        data.recordings = [];
      }
      dispatch(setRecordings(data.recordings));
    } else {
      dispatch(setMessage(`Error fetching recordings: ${response.statusText}`));
      dispatch(setShowMessage(true));
    }
  } catch (error: unknown) {
    dispatch(setMessage(`Error fetching recordings: ${(error as Error).message}`));
    dispatch(setShowMessage(true));
  }
};

export const fetchDeviceConfig = async (deviceInfo: { name: string; fileName: string }, dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/config/${deviceInfo.fileName}`);
    if (response.ok) {
      const data = await response.json();
      // Assuming yaml.dump is available globally or imported elsewhere if needed
      // For now, let's assume it's available or we'll need to import it.
      // If yaml is not available, this part needs adjustment.
      // For now, let's just log the data.
      console.log(`Config for ${deviceInfo.name}:`, data);
      // dispatch(setDeviceConfigs(prevConfigs => ({ ...prevConfigs, [deviceInfo.name]: yaml.dump(data, { indent: 2 }) })));
    } else {
      console.error(`Error fetching ${deviceInfo.fileName}: ${response.statusText}`);
    }
  } catch (error: unknown) {
    console.error(`Error fetching ${deviceInfo.fileName}: ${(error as Error).message}`);
  }
};

export const handleAction = async (
  endpoint: string,
  successMessage: string,
  errorMessage: string,
  device: DeviceStatus,
  dispatch: any,
  getState: () => RootState
) => {
  dispatch(setMessage("")); // Clear previous messages
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

      const { devices } = getState(); // Get current state
      if (device.type === "gigE") {
        dispatch(setDynamicCameras(devices.dynamicCameras.map((cam: DeviceStatus) =>
          cam.id === device.id
            ? { ...cam, status: data.status === "success" ? "running" : "stopped" }
            : cam
        )));
        if (devices.selectedCamera && devices.selectedCamera.id === device.id) {
          dispatch(setSelectedCameraAction(prev => prev ? { ...prev, status: data.status === "success" ? "running" : "stopped" } : null));
        }
      } else if (device.name === "Ouster Lidar") {
        dispatch(setOusterStatus(data.status === "success" ? "running" : "stopped"));
      }
      fetchRecordings(dispatch); // Refresh recordings
    } else {
      dispatch(setMessage(`${errorMessage}: ${data.detail || response.statusText}`));
    }
  } catch (error: unknown) {
    dispatch(setMessage(`${errorMessage}: ${(error as Error).message}`));
  }
};

export const handleSaveNotes = async (cameraId: string, notes: string, dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/notes`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ notes }),
    });
    const data = await response.json();
    if (response.ok) {
      dispatch(setMessage('Notes saved successfully.'));
      dispatch(setShowMessage(true));
    } else {
      dispatch(setMessage(`Error saving notes: ${data.detail}`));
      dispatch(setShowMessage(true));
    }
  } catch (error) {
    dispatch(setMessage(`Error saving notes: ${(error as Error).message}`));
    dispatch(setShowMessage(true));
  }
};

export const handleSetPublishingPreset = async (cameraId: string, presetName: string, dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/publishing_preset`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ preset_name: presetName }),
    });
    const data = await response.json();
    if (response.ok) {
      dispatch(setMessage('Publishing preset set successfully.'));
      dispatch(setShowMessage(true));
    } else {
      dispatch(setMessage(`Error setting publishing preset: ${data.detail}`));
      dispatch(setShowMessage(true));
    }
  } catch (error) {
    dispatch(setMessage(`Error setting publishing preset: ${(error as Error).message}`));
    dispatch(setShowMessage(true));
  }
};

// Helper to fetch status, used by fetchAllStatuses
const fetchStatus = async (endpoint: string, protocol?: string, cameraId?: string, dispatch?: any): Promise<string> => {
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
      if (dispatch) {
        dispatch(setMessage(`Error fetching status from ${endpoint}: ${response.statusText}`));
        dispatch(setShowMessage(true));
      }
      return "unknown";
    }
  } catch (error: unknown) {
    if (dispatch) {
      dispatch(setMessage(`Error fetching status from ${endpoint}: ${(error as Error).message}`));
      dispatch(setShowMessage(true));
    }
    return "unknown";
  }
};

// Function to fetch all statuses (static and dynamic)
export const fetchAllStatuses = async (dispatch: any, getState: () => RootState) => {
  const { devices } = getState();
  const staticDevicesInfo = [
    { name: "Ouster Lidar", path: "/ouster", apiEndpoint: "/driver/ouster", fileName: "ouster.yaml" },
  ]; // Define staticDevicesInfo here or pass it as an argument

  // Fetch status for static devices
  const ousterStatusResult = await fetchStatus("/driver/ouster/status", undefined, undefined, dispatch);
  dispatch(setOusterStatus(ousterStatusResult));

  // Fetch status for dynamic cameras
  const updatedDynamicCameras = await Promise.all(
    devices.dynamicCameras.map(async (cam: DeviceStatus) => {
      const status = await fetchStatus(
        `/driver/${cam.apiEndpoint.split('/').pop()}/status`,
        cam.type,
        cam.id,
        dispatch
      );
      return { ...cam, status };
    })
  );
  dispatch(setDynamicCameras(updatedDynamicCameras));

  // If selected camera's status changed, update selectedCamera
  if (devices.selectedCamera) {
    const updatedSelected = updatedDynamicCameras.find((c) => c.id === devices.selectedCamera.id);
    if (updatedSelected && updatedSelected.status !== devices.selectedCamera.status) {
      dispatch(setSelectedCameraAction(updatedSelected));
    }
  }
};

// Handler for creating a new camera
export const handleCreateCamera = async (newCameraId: string, newCameraProtocol: string, dispatch: any) => {
  dispatch(setMessage("")); // Clear previous messages
  if (!newCameraId.trim()) {
    dispatch(setMessage("Camera ID (IP) cannot be empty."));
    dispatch(setShowMessage(true));
    return;
  }

  if (newCameraProtocol === 'gigE') {
    const ipRegex = /^(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;
    if (!ipRegex.test(newCameraId)) {
      dispatch(setMessage("Invalid IP address format for GigE camera."));
      dispatch(setShowMessage(true));
      return;
    }
  }

  try {
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
      fetchCameras(dispatch, () => ({ devices: { dynamicCameras: [], selectedCamera: null } })); // Refresh the list of cameras (mock state for dispatch)
    } else {
      dispatch(setMessage(`Error creating camera: ${data.detail || response.statusText}`));
      dispatch(setShowMessage(true));
    }
  } catch (error: unknown) {
    dispatch(setMessage(`Error creating camera: ${(error as Error).message}`));
    dispatch(setShowMessage(true));
  }
};

// Handler for changing a feature value
export const handleFeatureChange = async (
  cameraId: string,
  featureGroupName: string,
  featureName: string,
  newValue: any,
  dispatch: any,
  getState: () => RootState
) => {
  dispatch(setMessage(""));
  try {
    // Placeholder for actual API call to update feature
    console.log(`Simulating update for feature ${featureName} in group ${featureGroupName} for camera ${cameraId} to ${newValue}`);

    // Update the local state immediately for better UX
    const { devices } = getState();
    const currentSelectedCamera = devices.selectedCamera;

    if (!currentSelectedCamera) {
      dispatch(setMessage("No camera selected."));
      dispatch(setShowMessage(true));
      return;
    }

    const updatedFeatures = currentSelectedCamera.features?.map(group => {
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

    const updatedCamera = { ...currentSelectedCamera, features: updatedFeatures };
    dispatch(setSelectedCameraAction(updatedCamera));

    // In a real scenario, you would call an API endpoint here:
    // const response = await fetch(`${ROS_API_URL}/update_feature`, { ... });
    // const data = await response.json();
    // if (response.ok) { ... } else { ... }

    // Re-fetch details to sync state from backend
    await fetchCameraDetails(cameraId, currentSelectedCamera.type || 'gigE', dispatch, getState);

    dispatch(setMessage(`Feature ${featureName} updated successfully!`));
    dispatch(setShowMessage(true));

  } catch (error: unknown) {
    dispatch(setMessage(`Error updating feature ${featureName}: ${(error as Error).message}`));
    dispatch(setShowMessage(true));
  }
};

export const handleSaveNotes = async (cameraId: string, notes: string, dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/notes`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ notes }),
    });
    const data = await response.json();
    if (response.ok) {
      dispatch(setMessage('Notes saved successfully.'));
      dispatch(setShowMessage(true));
    } else {
      dispatch(setMessage(`Error saving notes: ${data.detail}`));
      dispatch(setShowMessage(true));
    }
  } catch (error) {
    dispatch(setMessage(`Error saving notes: ${(error as Error).message}`));
    dispatch(setShowMessage(true));
  }
};

export const handleSetPublishingPreset = async (cameraId: string, presetName: string, dispatch: any) => {
  try {
    const response = await fetch(`${ROS_API_URL}/camera/${cameraId}/publishing_preset`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ preset_name: presetName }),
    });
    const data = await response.json();
    if (response.ok) {
      dispatch(setMessage('Publishing preset set successfully.'));
      dispatch(setShowMessage(true));
    } else {
      dispatch(setMessage(`Error setting publishing preset: ${data.detail}`));
      dispatch(setShowMessage(true));
    }
  } catch (error) {
    dispatch(setMessage(`Error setting publishing preset: ${(error as Error).message}`));
    dispatch(setShowMessage(true));
  }
};
