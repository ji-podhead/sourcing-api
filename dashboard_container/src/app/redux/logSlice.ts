import { createSlice, PayloadAction } from '@reduxjs/toolkit';

interface LogEntry {
  timestamp: string;
  message: string;
  level: 'info' | 'warn' | 'error';
}

// Adapting the state structure to be more like cameraSlice, with named properties
interface LogsState {
  messages: LogEntry[]; // Mimicking the structure of cameraSlice's properties
}

const initialState: LogsState = {
    messages: []
};

const logSlice = createSlice({
  name: 'logs',
  initialState,
  reducers: {
     addLogMessage(state, action: PayloadAction<string>) {
      const newLogEntry: LogEntry = {
        timestamp: new Date().toISOString(),
        message: action.payload,
        level: 'info', // Default level, can be adjusted if payload includes level
      };
      state.messages.push(newLogEntry);
    },
    // Reducer to set the logs, similar to setCameraFeatures
    setLogs(state, action: PayloadAction<LogEntry[]>) {
      state.messages = action.payload;
    },
    // Reducer to clear logs, similar to clearCameraFeatures
    clearLogs(state) {
      state.messages = []; // Set to empty object to match initialState
    },
  },
});

export const { addLogMessage, setLogs, clearLogs } = logSlice.actions;
export default logSlice.reducer;
