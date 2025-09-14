"use client";

import React from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { RootState, AppDispatch } from '../redux/store';
import { handleCreateCamera } from '../redux/thunks';
import { setIsDiscoveryPanelOpen } from '../redux/dashboardSlice';

const DiscoveryPanel: React.FC = () => {
  const dispatch: AppDispatch = useDispatch();
  const { discoveredCameras } = useSelector((state: RootState) => state.dashboard);

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex justify-center items-center z-40">
      <div className="bg-white p-6 rounded-lg shadow-xl w-1/2 max-w-4xl">
        <div className="flex justify-between items-center mb-4">
          <h2 className="text-2xl font-bold">Discovered Cameras</h2>
          <button onClick={() => dispatch(setIsDiscoveryPanelOpen(false))} className="text-2xl font-bold">&times;</button>
        </div>
        <div className="max-h-96 overflow-y-auto">
          <table className="w-full text-left">
            <thead>
              <tr>
                <th className="p-2 border-b">Name</th>
                <th className="p-2 border-b">IP Address</th>
                <th className="p-2 border-b">Device ID</th>
                <th className="p-2 border-b">Status</th>
                <th className="p-2 border-b">Actions</th>
              </tr>
            </thead>
            <tbody>
              {discoveredCameras.length > 0 ? (
                discoveredCameras.map((cam, index) => (
                  <tr key={index}>
                    <td className="p-2 border-b">{cam.name}</td>
                    <td className="p-2 border-b">{cam.ip_address}</td>
                    <td className="p-2 border-b">{cam.device_id}</td>
                    <td className="p-2 border-b">
                      {cam.is_in_db ? (
                        <span className="text-green-600">Already in DB</span>
                      ) : (
                        <span className="text-gray-500">Not in DB</span>
                      )}
                    </td>
                    <td className="p-2 border-b">
                      {!cam.is_in_db && (
                        <button
                          onClick={() => dispatch(handleCreateCamera({ identifier: cam.name }))}
                          className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-1 px-3 rounded text-sm"
                        >
                          Add
                        </button>
                      )}
                    </td>
                  </tr>
                ))
              ) : (
                <tr>
                  <td colSpan={5} className="text-center p-4">No cameras found on the network.</td>
                </tr>
              )}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
};

export default DiscoveryPanel;