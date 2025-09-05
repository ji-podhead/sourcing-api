import { createSlice, PayloadAction } from '@reduxjs/toolkit';
import { Preset } from '../app';

interface DashboardState {
  selectedFeatureGroup: string | null;
  activeTab: 'features' | 'presets';
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
};

const dashboardSlice = createSlice({
  name: 'dashboard',
  initialState,
  reducers: {
    setSelectedFeatureGroup(state, action: PayloadAction<string | null>) {
      state.selectedFeatureGroup = action.payload;
    },
    setActiveTab(state, action: PayloadAction<'features' | 'presets'>) {
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
} = dashboardSlice.actions;
export default dashboardSlice.reducer;
