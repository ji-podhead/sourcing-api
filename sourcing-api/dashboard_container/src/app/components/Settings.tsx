"use client";

import React from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { RootState } from '../../redux/store';
import { setIsSettingsOpen, setRefreshInterval } from '../../redux/dashboardSlice';

const Settings = () => {
  const dispatch = useDispatch();
  const refreshInterval = useSelector((state: RootState) => state.dashboard.refreshInterval);
  const isSettingsOpen = useSelector((state: RootState) => state.dashboard.isSettingsOpen);

  return (
    <div>
      <h2 className="text-xl font-semibold mb-4">Settings</h2>
      <div>
        <label htmlFor="refreshInterval" className="mr-2 font-medium">Refresh Interval:</label>
        <select
          id="refreshInterval"
          value={refreshInterval}
          onChange={(e) => dispatch(setRefreshInterval(Number(e.target.value)))}
          className="p-2 border rounded-md w-full"
        >
          <option value={2000}>2 seconds</option>
          <option value={4000}>4 seconds</option>
          <option value={5000}>5 seconds</option>
          <option value={10000}>10 seconds</option>
          <option value={60000}>1 minute</option>
          <option value={120000}>2 minutes</option>
          <option value={180000}>3 minutes</option>
          <option value={300000}>5 minutes</option>
          <option value={600000}>10 minutes</option>
        </select>
      </div>
    </div>
  );
};

export default Settings;
