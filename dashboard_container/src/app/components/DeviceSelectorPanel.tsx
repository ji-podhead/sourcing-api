"use client";

import React from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { RootState, AppDispatch } from '../redux/store';
import { fetchCameraDetails, handleCreateCamera, handleAction } from '../redux/thunks';
import { setSelectedCamera } from '../redux/devicesSlice';
import { setNewCameraId, setNewCameraProtocol } from '../redux/dashboardSlice';

const DeviceSelectorPanel: React.FC = () => {
  const dispatch: AppDispatch = useDispatch();
  const { dynamicCameras, selectedCamera } = useSelector((state: RootState) => state.devices);
  const { newCameraId, newCameraProtocol } = useSelector((state: RootState) => state.dashboard);

  const handleSelectCamera = (camId: string) => {
    const cam = dynamicCameras.find(c => c.id === camId);
    if (cam) {
      dispatch(setSelectedCamera(cam));
    } else {
      dispatch(setSelectedCamera(null));
    }
  };

  return (
    <div className="flex items-center space-x-4">
      <div style={{ display: 'flex', gap: '10px', alignItems: 'center', backgroundColor: '#f9f9f9', padding: '10px', borderRadius: '8px' }}>
        <h3 className="text-lg font-semibold">Add a Device</h3>
        <input
          type="text"
          placeholder="e.g., 192.168.1.100"
          style={{ padding: '8px', border: '1px solid #ccc', borderRadius: '4px' }}
          value={newCameraId}
          onChange={e => dispatch(setNewCameraId(e.target.value))}
        />
        <select
          style={{ padding: '8px', border: '1px solid #ccc', borderRadius: '4px' }}
          value={newCameraProtocol}
          onChange={e => dispatch(setNewCameraProtocol(e.target.value))}
        >
          <option value="gigE">GigE</option>
          <option value="gmsl2">GMSL2</option>
        </select>
        <button
          onClick={() => dispatch(handleCreateCamera({ newCameraId, newCameraProtocol }))}
          style={{ padding: '10px 15px', backgroundColor: '#007bff', color: 'white', border: 'none', borderRadius: '4px', cursor: 'pointer' }}
        >
          Add
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
            {cam.identifier} ({cam.type})
          </option>
        ))}
      </select>
      {selectedCamera && (
        <>
          <div className={`w-3 h-3 rounded-full ${selectedCamera.status === 'running' ? 'bg-green-500' : 'bg-red-500'}`}></div>
          <span className="text-sm text-black">Temp: N/A</span>
          <button
            onClick={() => dispatch(handleAction({
              endpoint: `${selectedCamera.apiEndpoint}/stop`,
              successMessage: `${selectedCamera.identifier} driver stopped.`,
              errorMessage: `Failed to stop ${selectedCamera.identifier} driver`,
              device: selectedCamera
            }))}
            className="bg-red-500 hover:bg-red-700 text-white font-bold py-2 px-4 rounded disabled:opacity-50 disabled:cursor-not-allowed"
            disabled={selectedCamera.status === "stopped" || !selectedCamera.apiEndpoint}
          >
            Stop
          </button>
          <button
            onClick={() => dispatch(handleAction({
              endpoint: `${selectedCamera.apiEndpoint}/start`,
              successMessage: `${selectedCamera.identifier} publishing started.`,
              errorMessage: `Failed to start ${selectedCamera.identifier} publishing`,
              device: selectedCamera
            }))}
            className="bg-purple-500 hover:bg-purple-700 text-white font-bold py-2 px-4 rounded disabled:opacity-50 disabled:cursor-not-allowed"
            disabled={selectedCamera.status === "running" || !selectedCamera.apiEndpoint}
          >
            Publish
          </button>
        </>
      )}
    </div>
  );
};

export default DeviceSelectorPanel;