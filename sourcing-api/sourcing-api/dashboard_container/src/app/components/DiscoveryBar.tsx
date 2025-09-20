"use client";

import React, { useEffect } from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { RootState, AppDispatch } from '../redux/store';
import { discoverAndFetchAll, handleCreateCamera, handleAction, fetchCameraDetails } from '../redux/thunks';
import { setSelectedCamera } from '../redux/devicesSlice';

const DiscoveryBar: React.FC = () => {
  const dispatch: AppDispatch = useDispatch();
  const { allCameras } = useSelector((state: RootState) => state.dashboard);
  const { selectedCamera } = useSelector((state: RootState) => state.devices);

  const handleSelectCamera = (cam: any) => {
    dispatch(setSelectedCamera(cam));
    if (cam.is_in_db) {
      dispatch(fetchCameraDetails(cam.id));
    }
  };

  return (
    <div className="w-full bg-white p-4 rounded-lg shadow-md mb-8">
      <h2 className="text-2xl font-semibold mb-4">Cameras</h2>
      <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-4">
        {allCameras.map((cam) => (
          <div
            key={cam.id || cam.camera_name}
            className={`p-4 rounded-lg cursor-pointer ${selectedCamera?.id === cam.id ? 'bg-blue-200' : 'bg-gray-100'} ${!cam.isDiscovered ? 'opacity-50' : ''}`}
            onClick={() => handleSelectCamera(cam)}
          >
            <h3 className="font-bold">{cam.camera_name}</h3>
            <p>IP: {cam.ip_address || 'N/A'}</p>
            <p>Status: {cam.isDiscovered ? 'Online' : 'Offline'}</p>
            <p>Temp: {cam.temperature || 'N/A'} °C</p>
            <div className="flex space-x-2 mt-2">
              {cam.is_in_db ? (
                <>
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      dispatch(handleAction({ endpoint: `/camera/${cam.id}/start`, successMessage: 'Camera started', errorMessage: 'Failed to start camera' }));
                    }}
                    className="bg-green-500 hover:bg-green-700 text-white font-bold py-1 px-2 rounded text-sm"
                  >
                    Start
                  </button>
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      dispatch(handleAction({ endpoint: `/camera/${cam.id}/stop`, successMessage: 'Camera stopped', errorMessage: 'Failed to stop camera' }));
                    }}
                    className="bg-red-500 hover:bg-red-700 text-white font-bold py-1 px-2 rounded text-sm"
                  >
                    Stop
                  </button>
                </>
              ) : (
                <button
                  onClick={(e) => {
                    e.stopPropagation();
                    dispatch(handleCreateCamera({ identifier: cam.camera_name }));
                  }}
                  className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-1 px-2 rounded text-sm"
                >
                  Add
                </button>
              )}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default DiscoveryBar;
