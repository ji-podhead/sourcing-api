import { createSlice, PayloadAction } from '@reduxjs/toolkit';
import { CameraFeatureGroup } from '../types';

interface CameraState {
  deviceControl: CameraFeatureGroup | {};
  ptpControl: CameraFeatureGroup | {};
  imageFormatControl: CameraFeatureGroup | {};
  analogControl: CameraFeatureGroup | {};
  acquisitionControl: CameraFeatureGroup | {};
}

const initialState: CameraState = {
  deviceControl: {},
  ptpControl: {},
  imageFormatControl: {},
  analogControl: {},
  acquisitionControl: {},
};

const cameraSlice = createSlice({
  name: 'camera',
  initialState,
  reducers: {
    setCameraFeatures(state, action: PayloadAction<CameraFeatureGroup[]>) {
      const features = action.payload;
      state.deviceControl = features.find(featureGroup => featureGroup.name === "DeviceControl") || {};
      state.ptpControl = features.find(featureGroup => featureGroup.name === "PtpControl") || {};
      state.imageFormatControl = features.find(featureGroup => featureGroup.name === "ImageFormatControl") || {};
      state.analogControl = features.find(featureGroup => featureGroup.name === "AnalogControl") || {};
      state.acquisitionControl = features.find(featureGroup => featureGroup.name === "AcquisitionControl") || {};
    },
    clearCameraFeatures(state) {
      state.deviceControl = {};
      state.ptpControl = {};
      state.imageFormatControl = {};
      state.analogControl = {};
      state.acquisitionControl = {};
    }
  },
});

export const { setCameraFeatures, clearCameraFeatures } = cameraSlice.actions;
export default cameraSlice.reducer;
