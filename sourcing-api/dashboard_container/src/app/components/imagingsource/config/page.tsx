"use client";

import { useState, useEffect } from 'react';
import yaml from 'js-yaml'; // Import js-yaml

export default function ImagingSourceConfigPage() {
  const [config, setConfig] = useState<any>({});
  const [loading, setLoading] = useState(true);
  const [message, setMessage] = useState('');
  const [isEditing, setIsEditing] = useState(false);
  const [editedConfig, setEditedConfig] = useState<string>('');

  const API_BASE_URL = process.env.NEXT_PUBLIC_API_BASE_URL || "http://localhost:8000";

  useEffect(() => {
    fetchConfig();
  }, []);

  const fetchConfig = async () => {
    setLoading(true);
    try {
      const response = await fetch(`${API_BASE_URL}/config/imaging_source`);
      const data = await response.json();
      setConfig(data);
      setEditedConfig(yaml.dump(data)); // Convert JSON to YAML string for editing
    } catch (error) {
      console.error("Error fetching ImagingSource config:", error);
      setMessage("Failed to fetch ImagingSource configuration.");
    } finally {
      setLoading(false);
    }
  };

  const handleEditChange = (event: React.ChangeEvent<HTMLTextAreaElement>) => {
    setEditedConfig(event.target.value);
  };

  const saveConfig = async () => {
    setLoading(true);
    setMessage('');
    try {
      const jsonConfig = yaml.load(editedConfig); // Convert YAML string back to JSON
      const response = await fetch(`${API_BASE_URL}/config/imaging_source`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(jsonConfig),
      });
      const data = await response.json();
      setMessage(data.message);
      if (data.status === "success") {
        setConfig(jsonConfig);
        setIsEditing(false);
      }
    } catch (error) {
      console.error("Error saving ImagingSource config:", error);
      setMessage("Failed to save ImagingSource configuration. Please check YAML syntax.");
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="h-full w-full flex flex-col items-center justify-center p-4">
      <h2 className="text-2xl font-semibold mb-4">ImagingSource Configuration</h2>
      {loading ? (
        <p>Loading configuration...</p>
      ) : (
        <>
          {isEditing ? (
            <div className="w-full max-w-2xl">
              <textarea
                className="w-full h-64 p-2 border rounded-md bg-gray-800 text-white font-mono"
                value={editedConfig}
                onChange={handleEditChange}
              />
              <div className="flex space-x-4 mt-4">
                <button
                  onClick={saveConfig}
                  disabled={loading}
                  className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                    loading ? 'bg-gray-400 cursor-not-allowed' : 'bg-blue-600 hover:bg-blue-700'
                  }`}
                >
                  Save Configuration
                </button>
                <button
                  onClick={() => setIsEditing(false)}
                  disabled={loading}
                  className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                    loading ? 'bg-gray-400 cursor-not-allowed' : 'bg-gray-600 hover:bg-gray-700'
                  }`}
                >
                  Cancel
                </button>
              </div>
            </div>
          ) : (
            <div className="w-full max-w-2xl">
              <pre className="bg-gray-800 text-white p-4 rounded-md overflow-auto max-h-96">
                {yaml.dump(config)}
              </pre>
              <button
                onClick={() => setIsEditing(true)}
                disabled={loading}
                className={`mt-4 px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${
                  loading ? 'bg-gray-400 cursor-not-allowed' : 'bg-blue-600 hover:bg-blue-700'
                }`}
              >
                Edit Configuration
              </button>
            </div>
          )}
          {message && <p className="mt-4 text-sm text-gray-600">{message}</p>}
        </>
      )}
    </div>
  );
}
