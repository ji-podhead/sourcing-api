import { createSlice, PayloadAction } from '@reduxjs/toolkit';

// DeviceStatus type from your page.tsx
export interface DeviceStatus {
  id: string;
  identifier: string;
  type: string;
  config: any;
  status: string;
  features?: any[];
  name: string;
  path: string;
  apiEndpoint: string;
  fileName: string;
  user_notes?: string;
  publishing_preset?: string;
}

interface DevicesState {
  dynamicCameras: DeviceStatus[];
  selectedCamera: DeviceStatus | null;
  ousterStatus: string;
}

const initialState: DevicesState = {
  dynamicCameras: [],
  selectedCamera: null,
  ousterStatus: 'stopped',
};

const devicesSlice = createSlice({
  name: 'devices',
  initialState,
  reducers: {
    setDynamicCameras(state, action: PayloadAction<DeviceStatus[]>) {
      state.dynamicCameras = action.payload;
    },
    setSelectedCamera(state, action: PayloadAction<DeviceStatus | null>) {
      state.selectedCamera = action.payload;
    },
    setOusterStatus(state, action: PayloadAction<string>) {
      state.ousterStatus = action.payload;
    },
  },
});

export const { setDynamicCameras, setSelectedCamera, setOusterStatus } = devicesSlice.actions;
export default devicesSlice.reducer;
