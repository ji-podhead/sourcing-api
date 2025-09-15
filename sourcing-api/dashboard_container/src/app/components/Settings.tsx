"use client";

import React from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { RootState } from '../redux/store';
import { setDiscoveryInterval as setDiscoveryIntervalAction, setRefreshInterval as setRefreshIntervalAction } from '../redux/dashboardSlice';
import { setIntervals } from '../redux/thunks';
import { AppDispatch } from '../redux/store';

const Settings = () => {
  const dispatch: AppDispatch = useDispatch();
  const { discoveryInterval, refreshInterval } = useSelector((state: RootState) => state.dashboard);

  const handleDiscoveryIntervalChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newDiscoveryInterval = Number(e.target.value);
    dispatch(setDiscoveryIntervalAction(newDiscoveryInterval));
    dispatch(setIntervals({ discovery_interval: newDiscoveryInterval, refresh_interval: refreshInterval }));
  };

  const handleRefreshIntervalChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newRefreshInterval = Number(e.target.value);
    dispatch(setRefreshIntervalAction(newRefreshInterval));
    dispatch(setIntervals({ discovery_interval: discoveryInterval, refresh_interval: newRefreshInterval }));
  };

  return (
    <div>
      <h2 className="text-xl font-semibold mb-4">Settings</h2>
      <div className="space-y-4">
        <div>
          <label htmlFor="discoveryInterval" className="mr-2 font-medium">Discovery Interval:</label>
          <select
            id="discoveryInterval"
            value={discoveryInterval}
            onChange={handleDiscoveryIntervalChange}
            className="p-2 border rounded-md w-full"
          >
            <option value={2000}>2 seconds</option>
            <option value={4000}>4 seconds</option>
            <option value={5000}>5 seconds</option>
            <option value={10000}>10 seconds</option>
            <option value={60000}>1 minute</option>
          </select>
        </div>
        <div>
          <label htmlFor="refreshInterval" className="mr-2 font-medium">Selected Camera Refresh Interval:</label>
          <select
            id="refreshInterval"
            value={refreshInterval}
            onChange={handleRefreshIntervalChange}
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
    </div>
  );
};

export default Settings;
