import { configureStore } from '@reduxjs/toolkit';
import devicesReducer from './devicesSlice';
import dashboardReducer from './dashboardSlice';

export const store = configureStore({
  reducer: {
    devices: devicesReducer,
    dashboard: dashboardReducer,
  },
});

export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;

// Add this for correct typing in useSelector
declare module 'react-redux' {
  interface DefaultRootState extends RootState {}
}
