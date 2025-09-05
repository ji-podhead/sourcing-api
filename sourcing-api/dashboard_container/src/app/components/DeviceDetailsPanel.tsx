import React, { useState, useEffect } from 'react';
import { CameraFeature, CameraFeatureGroup } from '../types';

interface DeviceDetailsPanelProps {
  selectedCamera: any;
  editingNotes: string;
  setEditingNotes: (notes: string) => void;
  handleSaveNotes: () => void;
  presets: any[];
  handleSetPublishingPreset: (presetName: string) => void;
}

const DeviceDetailsPanel: React.FC<DeviceDetailsPanelProps> = ({
  selectedCamera,
  editingNotes,
  setEditingNotes,
  handleSaveNotes,
  presets,
  handleSetPublishingPreset,
}) => {
  const [deviceControl, setDeviceControl] = useState<any>({});
  const [PtpControl, setPtpControl] = useState<any>({});
  const [ImageFormatControl, setImageFormatControl] = useState<any>({});
  const [AnalogControl, setAnalogControl] = useState<any>({});
  const [AcquisitionControl, setAcquisitionControl] = useState<any>({});

  useEffect(() => {
    if (selectedCamera && selectedCamera.features) {
      // Safely find feature groups and provide a default empty object if not found
      setDeviceControl(selectedCamera.features.find(featureGroup => featureGroup.name === "DeviceControl") || {});
      setPtpControl(selectedCamera.features.find(featureGroup => featureGroup.name === "PtpControl") || {});
      setImageFormatControl(selectedCamera.features.find(featureGroup => featureGroup.name === "ImageFormatControl") || {});
      setAnalogControl(selectedCamera.features.find(featureGroup => featureGroup.name === "AnalogControl") || {});
      setAcquisitionControl(selectedCamera.features.find(featureGroup => featureGroup.name === "AcquisitionControl") || {});
    } else {
      // Reset to default empty objects if selectedCamera or its features are not available
      setDeviceControl({});
      setPtpControl({});
      setImageFormatControl({});
      setAnalogControl({});
      setAcquisitionControl({});
    }
  }, [selectedCamera]);

  const getFeatureValue = (group: CameraFeatureGroup, featureName: string) => {
    if (group && group.features) {
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
      <p><strong>Identifier:</strong> {selectedCamera.identifier}</p>
      <p><strong>Type:</strong> {selectedCamera.type}</p>
      <p><strong>Status:</strong> <span className={`font-bold ${selectedCamera.status === 'running' ? 'text-green-600' : 'text-red-600'}`}>{selectedCamera.status}</span></p>
      <div>
        <h4 className="font-semibold mt-4">Device Control</h4>
        <p>Temperature: {getFeatureValue(deviceControl, 'DeviceTemperature')}</p>
        <h4 className="font-semibold mt-4">PTP Control</h4>
        <p>PTP Status: {getFeatureValue(PtpControl, 'PtpStatus')}</p>
        <h4 className="font-semibold mt-4">Image Format Control</h4>
        <p>Pixel Format: {getFeatureValue(ImageFormatControl, 'PixelFormat')}</p>
        <p>Sensor Width: {getFeatureValue(ImageFormatControl, 'SensorWidth')}</p>
        <p>Width: {getFeatureValue(ImageFormatControl, 'Width')}</p>
        <p>Sensor Height: {getFeatureValue(ImageFormatControl, 'SensorHeight')}</p>
        <p>Height: {getFeatureValue(ImageFormatControl, 'Height')}</p>
        <h4 className="font-semibold mt-4">Analog Control</h4>
        <p>Gain Auto: {getFeatureValue(AnalogControl, 'GainAuto')}</p>
        <p>Gain Auto Lower Limit: {getFeatureValue(AnalogControl, 'GainAutoLowerLimit')}</p>
        <p>Gain Auto Upper Limit: {getFeatureValue(AnalogControl, 'GainAutoUpperLimit')}</p>
        <p>Gain: {getFeatureValue(AnalogControl, 'Gain')}</p>
        <p>Gamma: {getFeatureValue(AnalogControl, 'Gamma')}</p>
        <p>Black Level: {getFeatureValue(AnalogControl, 'BlackLevel')}</p>
        <h4 className="font-semibold mt-4">Acquisition Control</h4>
        <p>Exposure Auto: {getFeatureValue(AcquisitionControl, 'ExposureAuto')}</p>
        <p>Acquisition Frame Rate: {getFeatureValue(AcquisitionControl, 'AcquisitionFrameRate')}</p>
        <p>Acquisition Mode: {getFeatureValue(AcquisitionControl, 'AcquisitionMode')}</p>
        <p>Exposure Auto Lower Limit: {getFeatureValue(AcquisitionControl, 'ExposureAutoLowerLimit')}</p>
        <p>Exposure Auto Reference: {getFeatureValue(AcquisitionControl, 'ExposureAutoReference')}</p>
        <p>Exposure Auto Upper Limit: {getFeatureValue(AcquisitionControl, 'ExposureAutoUpperLimit')}</p>
        <h4 className="font-semibold mt-4">User Notes</h4>
        <textarea
          value={editingNotes}
          onChange={e => setEditingNotes(e.target.value)}
          className="w-full p-2 border rounded-md mt-1"
          rows={4}
          placeholder="Enter notes for this device..."
        />
        <button onClick={handleSaveNotes} className="bg-blue-500 text-white p-2 rounded-md mt-2">Save Notes</button>
        <h4 className="font-semibold mt-4">Publishing Preset</h4>
        <div className="flex items-center space-x-2 mt-1">
          <select
            value={selectedCamera.publishing_preset || ''}
            onChange={e => handleSetPublishingPreset(e.target.value)}
            className="w-full p-2 border rounded-md"
          >
            <option value="">-- None --</option>
            {presets.map(p => <option key={p.name} value={p.name}>{p.name}</option>)}
          </select>
        </div>
      </div>
    </div>
  );
};

export default DeviceDetailsPanel;
