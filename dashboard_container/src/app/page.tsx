"use client";

import React, { useState, useEffect, useRef } from "react";
import dynamic from 'next/dynamic';
import { useWindowSize } from './handlers/WindowSizeContext';
import { useSelector, useDispatch } from 'react-redux';
import { RootState, AppDispatch } from './redux/store';
import {
  setActiveTab,
  setIsSettingsOpen,
  setShowMessage,
  setEditingNotes,
} from './redux/dashboardSlice';
import {
  fetchCameras,
  fetchAllStatuses,
  fetchRecordings,
  fetchPresets,
  handleSaveNotes,
  handleSetPublishingPreset,
  handleFeatureChange,
  handleCreatePreset,
  handleUpdatePreset,
  handleDeletePreset,
  applyPreset,
  connectToLogs,
} from './redux/thunks';
import Sidebar from "./components/Sidebar";
import FeaturePanel from "./components/FeaturePanel";
import PresetsPanel from "./components/PresetsPanel";
import DeviceSelectorPanel from "./components/DeviceSelectorPanel";
import Settings from "./components/Settings";
import DeviceDetailsPanel from "./components/DeviceDetailsPanel";

const WebTerminal = dynamic(() => import("./components/WebTerminal"), { ssr: false });
const TERMINAL_WEBSOCKET_URL = "ws://localhost:8000/terminal";

export default function Home() {
  const { height } = useWindowSize();
  const dispatch: AppDispatch = useDispatch();
  const {
    selectedCamera,
    ousterStatus,
  } = useSelector((state: RootState) => state.devices);
  const {
    selectedFeatureGroup,
    activeTab,
    isSettingsOpen,
    editingNotes,
    presets,
    presetForm,
    presetFormError,
    selectedPresetForEditing,
    refreshInterval,
    message,
    showMessage,
  } = useSelector((state: RootState) => state.dashboard);
  useEffect(() => {
    dispatch(fetchCameras());
    dispatch(fetchAllStatuses());
    dispatch(fetchRecordings());
    dispatch(connectToLogs());
    const statusInterval = setInterval(() => {
      dispatch(fetchAllStatuses());
      dispatch(fetchCameras());
    }, refreshInterval);
    const recordingInterval = setInterval(() => dispatch(fetchRecordings()), 10000);

    return () => {
      clearInterval(statusInterval);
      clearInterval(recordingInterval);
    };
  }, [dispatch, refreshInterval]);

  useEffect(() => {
    if (activeTab === 'presets' && selectedCamera) {
      dispatch(fetchPresets(selectedCamera.id));
    }
  }, [dispatch, activeTab, selectedCamera]);

  useEffect(() => {
    dispatch(connectToLogs());
  }, [dispatch]);

  return (
    <div className="w-full h-full text-black bg-gray-100 p-8">
      <Sidebar isOpen={isSettingsOpen} onClose={() => dispatch(setIsSettingsOpen(false))}>
        <Settings />
      </Sidebar>
      <button
        onClick={() => dispatch(setIsSettingsOpen(true))}
        className="fixed top-4 right-4 bg-blue-600 text-white p-2 rounded-md shadow-lg z-30"
      >
        Settings
      </button>
      <div className="flex justify-center items-center mb-8">
        <h1 className="text-4xl font-bold text-center">Dashboard</h1>
      </div>

      {message && showMessage && (
        <div className="bg-blue-100 border-l-4 border-blue-500 text-blue-700 p-4 mb-6 flex justify-between items-center" role="alert">
          <div>
            <p className="font-bold">Info</p>
            <p>{message}</p>
          </div>
          <button
            className="ml-4 text-blue-700 hover:text-blue-900 font-bold text-xl"
            onClick={() => dispatch(setShowMessage(false))}
            aria-label="Close alert"
          >
            &times;
          </button>
        </div>
      )}

      <div className="w-full bg-white p-6 rounded-lg shadow-md mb-8 h-[50%] overflow-scroll">
        <DeviceSelectorPanel />
      </div>

      {selectedCamera ? (
        <div className="flex flex-col md:flex-row gap-4 w-full h-full overflow-scroll " style={{ height: height * 0.5 }}>
          <DeviceDetailsPanel />
          <div className="w-full md:w-[80%] bg-gray-50 p-4 rounded-md">
            <div className="flex overflow-x-auto space-x-2 mb-4 pb-2 border-b">
              <button
                onClick={() => dispatch(setActiveTab('features'))}
                className={`px-4 py-2 rounded-md text-sm font-medium whitespace-nowrap ${activeTab === 'features' ? 'bg-blue-600 text-white' : 'bg-gray-200 hover:bg-gray-300'}`}
              >UNIQUE (device_identifier, name) -- Ensure unique preset names per device
                Features
              </button>
              <button
                onClick={() => {
                  dispatch(setActiveTab('presets'));
                  if (selectedCamera) {
                    dispatch(fetchPresets(selectedCamera.id));
                  }
                }}
                className={`px-4 py-2 rounded-md text-sm font-medium whitespace-nowrap ${activeTab === 'presets' ? 'bg-blue-600 text-white' : 'bg-gray-200 hover:bg-gray-300'}`}
              >
                Presets
              </button>
              <button
                onClick={() => dispatch(setActiveTab('calibration'))}
                className={`px-4 py-2 rounded-md text-sm font-medium whitespace-nowrap ${activeTab === 'calibration' ? 'bg-blue-600 text-white' : 'bg-gray-200 hover:bg-gray-300'}`}
              >
                Calibration
              </button>
            </div>

            {activeTab === 'features' ? (
              <FeaturePanel />
            ) : activeTab === 'presets' ? (
              <PresetsPanel />
            ) : (
              <div>
                <h2 className="text-2xl font-semibold mb-4">Calibration</h2>
                {selectedCamera && selectedCamera.publishing_preset ? (
                  <div>
                    <p>
                      The currently active preset for publishing is: <strong>{selectedCamera.publishing_preset}</strong>
                    </p>
                    <button
                      onClick={() => dispatch(applyPreset({ cameraId: selectedCamera.id, presetName: selectedCamera.publishing_preset! }))}
                      className="bg-blue-500 hover:bg-blue-700 text-white font-bold py-2 px-4 rounded mt-4"
                    >
                      Re-apply Preset
                    </button>
                  </div>
                ) : (
                  <p>No publishing preset selected for this camera.</p>
                )}
              </div>
            )}
          </div>
        </div>
      ) : (
        <p className="text-center text-black">Select a device to view its data and settings.</p>
      )}

      <div className="w-full bg-white p-6 rounded-lg shadow-md mb-8">
        <h2 className="text-2xl font-semibold mb-4">Web Terminal</h2>
        <WebTerminal websocketUrl={TERMINAL_WEBSOCKET_URL} />
      </div>

      <div className="w-full bg-white p-6 rounded-lg shadow-md mb-8">
        <h2 className="text-2xl font-semibold mb-4">Webviz</h2>
        <div className="w-full h-96 border rounded-md">
          <iframe src="/webviz" className="w-full h-full"></iframe>
        </div>
      </div>
    </div>
  );
}
