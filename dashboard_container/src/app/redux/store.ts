import { configureStore } from '@reduxjs/toolkit';
import devicesReducer from './devicesSlice';
import dashboardReducer from './dashboardSlice';
import cameraReducer from './cameraSlice';
import logsReducer from './logSlice'; // Import from the correct file

export const store = configureStore({
  reducer: {
    devices: devicesReducer,
    dashboard: dashboardReducer,
    camera: cameraReducer,
    logs: logsReducer, // Use the imported logsReducer
  },
});

export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;

// Add this for correct typing in useSelector
declare module 'react-redux' {
  interface DefaultRootState extends RootState {}
}
