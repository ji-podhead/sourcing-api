import { createSlice, PayloadAction } from '@reduxjs/toolkit';
import { Preset, CameraFeature } from '../types';

interface DashboardState {
  selectedFeatureGroup: string | null;
  activeTab: 'features' | 'presets' | 'calibration';
  isSettingsOpen: boolean;
  editingNotes: string;
  presets: Preset[];
  selectedPresetForEditing: Preset | null;
  presetForm: { name: string; configuration: Record<string, Record<string, { type: string; value: any }>> };
  presetFormError: string;
  refreshInterval: number;
  recordingStatus: string;
  recordings: any[];
  message: string;
  showMessage: boolean;
  newCameraId: string;
  newCameraProtocol: string;
}

const initialState: DashboardState = {
  selectedFeatureGroup: null,
  activeTab: 'features',
  isSettingsOpen: false,
  editingNotes: '',
  presets: [],
  selectedPresetForEditing: null,
  presetForm: { name: '', configuration: {} },
  presetFormError: '',
  refreshInterval: 5000,
  recordingStatus: 'stopped',
  recordings: [],
  message: '',
  showMessage: true,
  newCameraId: '',
  newCameraProtocol: 'gigE',
};

const dashboardSlice = createSlice({
  name: 'dashboard',
  initialState,
  reducers: {
    setSelectedFeatureGroup(state, action: PayloadAction<string | null>) {
      state.selectedFeatureGroup = action.payload;
    },
    setActiveTab(state, action: PayloadAction<'features' | 'presets' | 'calibration'>) {
      state.activeTab = action.payload;
    },
    setIsSettingsOpen(state, action: PayloadAction<boolean>) {
      state.isSettingsOpen = action.payload;
    },
    setEditingNotes(state, action: PayloadAction<string>) {
      state.editingNotes = action.payload;
    },
    setPresets(state, action: PayloadAction<Preset[]>) {
      state.presets = action.payload;
    },
    setSelectedPresetForEditing(state, action: PayloadAction<Preset | null>) {
      state.selectedPresetForEditing = action.payload;
    },
    setPresetForm(state, action: PayloadAction<{ name: string; configuration: Record<string, Record<string, { type: string; value: any }>> }>) {
      state.presetForm = action.payload;
    },
    setPresetFormError(state, action: PayloadAction<string>) {
      state.presetFormError = action.payload;
    },
    setRefreshInterval(state, action: PayloadAction<number>) {
      state.refreshInterval = action.payload;
    },
    setRecordingStatus(state, action: PayloadAction<string>) {
      state.recordingStatus = action.payload;
    },
    setRecordings(state, action: PayloadAction<any[]>) {
      state.recordings = action.payload;
    },
    setMessage(state, action: PayloadAction<string>) {
      state.message = action.payload;
    },
    setShowMessage(state, action: PayloadAction<boolean>) {
      state.showMessage = action.payload;
    },
    setPresetFormName(state, action: PayloadAction<string>) {
      state.presetForm.name = action.payload;
    },
    togglePresetFeature(state, action: PayloadAction<{ feature: CameraFeature; groupName: string; isChecked: boolean }>) {
      const { feature, groupName, isChecked } = action.payload;
      const newConfig = { ...state.presetForm.configuration };
      if (isChecked) {
        if (!newConfig[groupName]) {
          newConfig[groupName] = {};
        }
        newConfig[groupName][feature.name] = {
          type: feature.type,
          value: feature.value,
        };
      } else {
        if (newConfig[groupName]) {
          delete newConfig[groupName][feature.name];
          if (Object.keys(newConfig[groupName]).length === 0) {
            delete newConfig[groupName];
          }
        }
      }
      state.presetForm.configuration = newConfig;
    },
    setPresetValue(state, action: PayloadAction<{ groupName: string; featureName: string; newValue: any }>) {
      const { groupName, featureName, newValue } = action.payload;
      if (state.presetForm.configuration[groupName] && state.presetForm.configuration[groupName][featureName]) {
        state.presetForm.configuration[groupName][featureName].value = newValue;
      }
    },
    cancelPresetEdit(state) {
      state.selectedPresetForEditing = null;
      state.presetForm = { name: '', configuration: {} };
      state.presetFormError = '';
    },
    loadPresetForEditing(state, action: PayloadAction<Preset>) {
      state.selectedPresetForEditing = action.payload;
      state.presetForm = { name: action.payload.name, configuration: { ...action.payload.configuration } };
      state.presetFormError = '';
      state.activeTab = 'presets';
    },
    setNewCameraId(state, action: PayloadAction<string>) {
      state.newCameraId = action.payload;
    },
    setNewCameraProtocol(state, action: PayloadAction<string>) {
      state.newCameraProtocol = action.payload;
    },
  },
});

export const {
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
  setPresetFormName,
  togglePresetFeature,
  setPresetValue,
  cancelPresetEdit,
  loadPresetForEditing,
  setNewCameraId,
  setNewCameraProtocol,
} = dashboardSlice.actions;
export default dashboardSlice.reducer;
