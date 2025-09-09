"use client";
import React from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { RootState, AppDispatch } from '../redux/store';
import { setEditingNotes } from '../redux/dashboardSlice';
import { handleSaveNotes, handleSetPublishingPreset, handleDeleteCamera } from '../redux/thunks';
import { CameraFeature, CameraFeatureGroup } from '../types';

const DeviceDetailsPanel: React.FC = () => {
  const dispatch: AppDispatch = useDispatch();
  const { selectedCamera } = useSelector((state: RootState) => state.devices);
  const { editingNotes, presets } = useSelector((state: RootState) => state.dashboard);
  const {
    deviceControl,
    ptpControl,
    imageFormatControl,
    analogControl,
    acquisitionControl,
  } = useSelector((state: RootState) => state.camera);

  const getFeatureValue = (group: CameraFeatureGroup | {}, featureName: string) => {
    if ('features' in group && group.features) {
      const feature = group.features.find((f: CameraFeature) => f.name === featureName);
      return feature ? feature.value : 'N/A';
    }
    return 'N/A';
  };

  if (!selectedCamera) return null;

  return (
    <div className="w-full md:w-[18%] bg-gray-50 p-4 rounded-md">
      <h3 className="text-xl font-semibold mb-4">Device Data</h3>
      <p><strong>ID:</strong> {selectedCamera.id}</p>
      <p><strong>Name:</strong> {selectedCamera.camera_name}</p>
      <p><strong>IP:</strong> {selectedCamera.camera_ip}</p>
      <p><strong>Type:</strong> {selectedCamera.type}</p>
      <p><strong>Status:</strong> <span className={`font-bold ${selectedCamera.status === 'running' ? 'text-green-600' : 'text-red-600'}`}>{selectedCamera.status}</span></p>
      <div>
        <h4 className="font-semibold mt-4">Device Control</h4>
        <p>Temperature: {getFeatureValue(deviceControl, 'DeviceTemperature')}</p>
        <h4 className="font-semibold mt-4">PTP Control</h4>
        <p>PTP Status: {getFeatureValue(ptpControl, 'PtpStatus')}</p>
        <h4 className="font-semibold mt-4">Image Format Control</h4>
        <p>Pixel Format: {getFeatureValue(imageFormatControl, 'PixelFormat')}</p>
        <p>Sensor Width: {getFeatureValue(imageFormatControl, 'SensorWidth')}</p>
        <p>Width: {getFeatureValue(imageFormatControl, 'Width')}</p>
        <p>Sensor Height: {getFeatureValue(imageFormatControl, 'SensorHeight')}</p>
        <p>Height: {getFeatureValue(imageFormatControl, 'Height')}</p>
        <h4 className="font-semibold mt-4">Analog Control</h4>
        <p>Gain Auto: {getFeatureValue(analogControl, 'GainAuto')}</p>
        <p>Gain Auto Lower Limit: {getFeatureValue(analogControl, 'GainAutoLowerLimit')}</p>
        <p>Gain Auto Upper Limit: {getFeatureValue(analogControl, 'GainAutoUpperLimit')}</p>
        <p>Gain: {getFeatureValue(analogControl, 'Gain')}</p>
        <p>Gamma: {getFeatureValue(analogControl, 'Gamma')}</p>
        <p>Black Level: {getFeatureValue(analogControl, 'BlackLevel')}</p>
        <h4 className="font-semibold mt-4">Acquisition Control</h4>
        <p>Exposure Auto: {getFeatureValue(acquisitionControl, 'ExposureAuto')}</p>
        <p>Acquisition Frame Rate: {getFeatureValue(acquisitionControl, 'AcquisitionFrameRate')}</p>
        <p>Acquisition Mode: {getFeatureValue(acquisitionControl, 'AcquisitionMode')}</p>
        <p>Exposure Auto Lower Limit: {getFeatureValue(acquisitionControl, 'ExposureAutoLowerLimit')}</p>
        <p>Exposure Auto Reference: {getFeatureValue(acquisitionControl, 'ExposureAutoReference')}</p>
        <p>Exposure Auto Upper Limit: {getFeatureValue(acquisitionControl, 'ExposureAutoUpperLimit')}</p>
        <h4 className="font-semibold mt-4">User Notes</h4>
        <textarea
          value={editingNotes}
          onChange={e => dispatch(setEditingNotes(e.target.value))}
          className="w-full p-2 border rounded-md mt-1"
          rows={4}
          placeholder="Enter notes for this device..."
        />
        <button onClick={() => dispatch(handleSaveNotes({ cameraId: selectedCamera.id, notes: editingNotes }))} 
        className="bg-blue-500 text-white p-2 rounded-md mt-2">Save Notes</button>        <h4 className="font-semibold mt-4">Publishing Preset</h4>
        <div className="flex items-center space-x-2 mt-1">
          <select
            value={selectedCamera.publishing_preset || ''}
            onChange={e => dispatch(handleSetPublishingPreset({ cameraId: selectedCamera.id, presetName: e.target.value }))}
            className="w-full p-2 border rounded-md"
          >
            <option value="">-- Default --</option>
            <option value="default">Default</option>
            {presets.map(p => <option key={p.name} value={p.name}>{p.name}</option>)}
          </select>
        </div>
        <div className="mt-4">
            <button
                onClick={() => {
                    if (window.confirm(`Are you sure you want to delete ${selectedCamera.camera_name}? This action cannot be undone.`)) {
                        dispatch(handleDeleteCamera(selectedCamera.id));
                    }
                }}
                className="w-full bg-red-600 hover:bg-red-800 text-white font-bold py-2 px-4 rounded"
            >
                Delete Camera
            </button>
        </div>
      </div>
    </div>
  );
};

export default DeviceDetailsPanel;
