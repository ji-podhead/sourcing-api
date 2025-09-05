"use client";

import { useState, useEffect } from 'react';

export default function ImagingSourceDriverPage() {
  const [isRunning, setIsRunning] = useState(false);
  const [loading, setLoading] = useState(true);
  const [message, setMessage] = useState('');

  const API_BASE_URL = process.env.NEXT_PUBLIC_API_BASE_URL || "http://localhost:8000";

  useEffect(() => {
    fetchStatus();
  }, []);

  const fetchStatus = async () => {
    setLoading(true);
    try {
      const response = await fetch(`${API_BASE_URL}/driver/camera/status`);
      const data = await response.json();
      setIsRunning(data.status === "running");
    } catch (error) {
      console.error("Error fetching Camera status:", error);
      setMessage("Failed to fetch Camera status.");
    } finally {
      setLoading(false);
    }
  };

  const startDriver = async () => {
    setLoading(true);
    setMessage('');
    try {
      const response = await fetch(`${API_BASE_URL}/driver/camera/start`, { method: 'POST' });
      const data = await response.json();
      setMessage(data.message);
      if (data.status === "success") {
        setIsRunning(true);
      }
    } catch (error) {
      console.error("Error starting Camera driver:", error);
      setMessage("Failed to start Camera driver.");
    } finally {
      setLoading(false);
    }
  };

  const stopDriver = async () => {
    setLoading(true);
    setMessage('');
    try {
      const response = await fetch(`${API_BASE_URL}/driver/camera/stop`, { method: 'POST' });
      const data = await response.json();
      setMessage(data.message);
      if (data.status === "success") {
        setIsRunning(false);
      }
    } catch (error) {
      console.error("Error stopping Camera driver:", error);
      setMessage("Failed to stop Camera driver.");
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="h-full w-full flex flex-col items-center justify-center p-4">
      <h2 className="text-2xl font-semibold mb-4">ImagingSource Driver Control</h2>
      {loading ? (
        <p>Loading status...</p>
      ) : (
        <>
          <p className="mb-4">Status: <span className={`font-bold ${isRunning ? 'text-green-500' : 'text-red-500'}`}>
            {isRunning ? "Running" : "Stopped"}
          </span></p>
          <div className="flex space-x-4">
            <button
              onClick={startDriver}
              disabled={isRunning || loading}
              className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                isRunning || loading ? 'bg-gray-400 cursor-not-allowed' : 'bg-green-600 hover:bg-green-700'
              }`}
            >
              Start ImagingSource Driver
            </button>
            <button
              onClick={stopDriver}
              disabled={!isRunning || loading}
              className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                !isRunning || loading ? 'bg-gray-400 cursor-not-allowed' : 'bg-red-600 hover:bg-red-700'
              }`}
            >
              Stop ImagingSource Driver
            </button>
          </div>
          {message && <p className="mt-4 text-sm text-gray-600">{message}</p>}
        </>
      )}
    </div>
  );
}
