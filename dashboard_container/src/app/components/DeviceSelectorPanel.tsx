"use client";

import React from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { RootState, AppDispatch } from '../redux/store';
import { fetchCameraDetails, handleCreateCamera, handleAction, discoverCameras } from '../redux/thunks';
import { setSelectedCamera } from '../redux/devicesSlice';
import { setNewCameraIdentifier, setIsDiscoveryPanelOpen } from '../redux/dashboardSlice';
import DiscoveryPanel from './DiscoveryPanel';

const DeviceSelectorPanel: React.FC = () => {
  const dispatch: AppDispatch = useDispatch();
  const { dynamicCameras, selectedCamera } = useSelector((state: RootState) => state.devices);
  const { newCameraIdentifier, isDiscoveryPanelOpen } = useSelector((state: RootState) => state.dashboard);

  const handleSelectCamera = (camId: string) => {
    const numericCamId = parseInt(camId, 10);
    if (isNaN(numericCamId)) {
        dispatch(setSelectedCamera(null));
        return;
    }
    const cam = dynamicCameras.find(c => c.id === numericCamId);
    if (cam) {
        dispatch(setSelectedCamera(cam));
        dispatch(fetchCameraDetails(numericCamId));
    } else {
        dispatch(setSelectedCamera(null));
    }
  };

  const handleDiscover = () => {
    dispatch(discoverCameras());
    dispatch(setIsDiscoveryPanelOpen(true));
  };

  return (
    <>
      <div className="flex items-center space-x-4">
        <div style={{ display: 'flex', gap: '10px', alignItems: 'center', backgroundColor: '#f9f9f9', padding: '10px', borderRadius: '8px' }}>
          <h3 className="text-lg font-semibold">Add a Device</h3>
          <input
            type="text"
            placeholder="Enter Name or IP"
            style={{ padding: '8px', border: '1px solid #ccc', borderRadius: '4px' }}
            value={newCameraIdentifier}
            onChange={e => dispatch(setNewCameraIdentifier(e.target.value))}
          />
          <button
            onClick={() => dispatch(handleCreateCamera({ identifier: newCameraIdentifier }))}
            style={{ padding: '10px 15px', backgroundColor: '#007bff', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
          >
            Add Manually
          </button>
          <button
            onClick={handleDiscover}
            style={{ padding: '10px 15px', backgroundColor: '#28a745', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
          >
            Discover Cameras
          </button>
        </div>
        <label htmlFor="selectCamera" className="font-medium">Select Device:</label>
        <select
          id="selectCamera"
          className="p-2 border rounded-md"
          value={selectedCamera?.id || ''}
          onChange={e => handleSelectCamera(e.target.value)}
        >
          <option value="">-- Select a Device --</option>
          {dynamicCameras.map(cam => (
            <option
              key={cam.id}
              value={cam.id}
            >
              {cam.camera_name} ({cam.camera_ip})
            </option>
          ))}
        </select>
        {selectedCamera && (
          <>
            <div className={`w-3 h-3 rounded-full ${selectedCamera.status === 'running' ? 'bg-green-500' : 'bg-red-500'}`}></div>
            <button
              onClick={() => dispatch(handleAction({
                endpoint: `/camera/${selectedCamera.id}/stop`,
                successMessage: `${selectedCamera.camera_name} driver stopped.`,
                errorMessage: `Failed to stop ${selectedCamera.camera_name} driver`
              }))}
              className="bg-red-500 hover:bg-red-700 text-white font-bold py-2 px-4 rounded"
            >
              Stop
            </button>
            <button
              onClick={() => dispatch(handleAction({
                endpoint: `/camera/${selectedCamera.id}/start`,
                successMessage: `${selectedCamera.camera_name} publishing started.`,
                errorMessage: `Failed to start ${selectedCamera.camera_name} publishing`
              }))}
              className="bg-purple-500 hover:bg-purple-700 text-white font-bold py-2 px-4 rounded"
            >
              Publish
            </button>
          </>
        )}
      </div>
      {isDiscoveryPanelOpen && <DiscoveryPanel />}
    </>
  );
};

export default DeviceSelectorPanel;