import React from 'react';

interface PresetsPanelProps {
  selectedCamera: any;
  presets: any[];
  presetForm: any;
  presetFormError: string;
  selectedPresetForEditing: any;
  activeTab: string;
  handleCreatePreset: () => void;
  handleUpdatePreset: () => void;
  handleDeletePreset: (presetName: string) => void;
  loadPresetForEditing: (preset: any) => void;
  handleCancelPresetEdit: () => void;
  handlePresetValueChange: (groupName: string, featureName: string, newValue: any) => void;
  handleTogglePresetFeature: (feature: any, groupName: string, isChecked: boolean) => void;
  dispatch: any;
  setPresetForm: (form: any) => void;
  setSelectedPresetForEditing: (preset: any) => void;
  setActiveTab: (tab: string) => void;
}

const PresetsPanel: React.FC<PresetsPanelProps> = ({
  selectedCamera,
  presets,
  presetForm,
  presetFormError,
  selectedPresetForEditing,
  activeTab,
  handleCreatePreset,
  handleUpdatePreset,
  handleDeletePreset,
  loadPresetForEditing,
  handleCancelPresetEdit,
  handlePresetValueChange,
  handleTogglePresetFeature,
  dispatch,
  setPresetForm,
  setSelectedPresetForEditing,
  setActiveTab,
}) => {
  return (
    <>
      <div className="mb-4 flex justify-between items-center">
        <h4 className="text-lg font-semibold">Manage Presets</h4>
        <div>
          <button
            onClick={() => {
              setPresetForm({ name: '', configuration: {} });
              setSelectedPresetForEditing(null);
              setActiveTab('presets');
            }}
            className="bg-green-500 hover:bg-green-700 text-white font-bold py-2 px-4 rounded"
          >
            Create New Preset
          </button>
        </div>
      </div>

      {presetFormError && (
        <p className="text-red-500 mb-4">{presetFormError}</p>
      )}

      {/* Preset Form for Creating/Editing */}
      {(!selectedPresetForEditing || activeTab === 'presets') && (
        <div className="border p-4 rounded-md mb-4 bg-gray-50">
          <h5 className="text-lg font-semibold mb-3">{selectedPresetForEditing ? 'Edit Preset' : 'Create New Preset'}</h5>
          <div className="space-y-3">
            <div>
              <label htmlFor="presetName" className="block text-sm font-medium text-gray-700">Preset Name</label>
              <input
                type="text"
                id="presetName"
                value={presetForm.name}
                onChange={e => dispatch(setPresetForm({ ...presetForm, name: e.target.value }))}
                className="mt-1 p-2 w-full border rounded-md"
                placeholder="Enter preset name"
              />
            </div>

            {/* Feature Selection for Preset */}
            <div className="space-y-2">
              <h4 className="text-md font-semibold">Features</h4>
              <div className="max-h-60 overflow-y-auto border p-2 rounded-md">
                {selectedCamera?.features?.map((group: any) => (
                  <div key={group.name}>
                    <h5 className="font-semibold mt-2 text-gray-600">{group.name}</h5>
                    {group.features.map((feature: any) => (
                      <div key={feature.name} className="flex items-center justify-between pl-2">
                        <label
                          htmlFor={`preset-feature-${feature.name}`}
                          className={`flex-grow ${!feature.is_writable ? 'line-through text-gray-400' : ''}`}
                        >
                          {feature.name}
                        </label>
                        <input
                          type="checkbox"
                          id={`preset-feature-${feature.name}`}
                          checked={!!presetForm.configuration[group.name]?.[feature.name]}
                          disabled={!feature.is_writable}
                          onChange={e => handleTogglePresetFeature(feature, group.name, e.target.checked)}
                        />
                      </div>
                    ))}
                  </div>
                ))}
              </div>
            </div>

            {/* Feature Value Editing Section */}
            {Object.keys(presetForm.configuration).length > 0 && (
              <div className="mt-4 pt-4 border-t">
                <h4 className="text-md font-semibold">Preset Values</h4>
                <div className="space-y-3 mt-2 max-h-60 overflow-y-auto">
                  {Object.keys(presetForm.configuration).map(groupName => (
                    Object.keys(presetForm.configuration[groupName]).map(featureName => {
                      const feature = selectedCamera?.features?.find((g: any) => g.name === groupName)?.features.find((f: any) => f.name === featureName);
                      if (!feature) return null;

                      const presetFeature = presetForm.configuration[groupName][featureName];

                      return (
                        <div key={`${groupName}-${featureName}`} className="flex flex-col">
                          <label className="text-sm font-medium text-gray-700">{featureName}</label>
                          {feature.type === "Enumeration" ? (
                            <select
                              value={presetFeature.value}
                              onChange={e => handlePresetValueChange(groupName, featureName, e.target.value)}
                              className="w-full p-2 border rounded-md mt-1"
                            >
                              {feature.options?.map((option: any) => (
                                <option key={option} value={option}>{option}</option>
                              ))}
                            </select>
                          ) : feature.type === "Boolean" ? (
                            <input
                              type="checkbox"
                              checked={presetFeature.value}
                              onChange={e => handlePresetValueChange(groupName, featureName, e.target.checked)}
                              className="h-4 w-4 text-blue-600 border-gray-300 rounded focus:ring-blue-500 mt-1"
                            />
                          ) : (
                            <input
                              type={feature.type === "Integer" || feature.type === "Float" ? "number" : "text"}
                              value={presetFeature.value}
                              onChange={e => handlePresetValueChange(groupName, featureName, e.target.value)}
                              className="w-full p-2 border rounded-md mt-1"
                              min={feature.min}
                              max={feature.max}
                            />
                          )}
                        </div>
                      );
                    })
                  ))}
                </div>
              </div>
            )}
          </div>
          <div className="flex space-x-4 mt-4">
            <button
              onClick={selectedPresetForEditing ? handleUpdatePreset : handleCreatePreset}
              className={`px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 ${presetFormError ? 'bg-red-500' : 'bg-blue-600 hover:bg-blue-700'}`}
            >
              {selectedPresetForEditing ? 'Update Preset' : 'Create Preset'}
            </button>
            <button
              onClick={handleCancelPresetEdit}
              className="px-6 py-3 rounded-lg text-white font-semibold transition-colors duration-200 bg-gray-600 hover:bg-gray-700"
            >
              Cancel
            </button>
          </div>
        </div>
      )}

      {/* List of Existing Presets */}
      <div className="border p-4 rounded-md bg-gray-50">
        <h4 className="text-lg font-semibold mb-3">Existing Presets</h4>
        {presets.length === 0 ? (
          <p>No presets found for this device.</p>
        ) : (
          <ul className="space-y-2">
            {presets.map(preset => (
              <li key={preset.name} className="flex items-center justify-between p-2 border-b last:border-b-0">
                <span>{preset.name}{preset.name === "Default" && <span className="text-xs text-gray-500 ml-2">(Read-only)</span>}</span>
                <div className="flex space-x-2">
                  <button
                    onClick={() => loadPresetForEditing(preset)}
                    disabled={preset.name === "Default"}
                    className={`font-bold py-1 px-3 rounded text-sm text-white ${preset.name === "Default" ? "bg-gray-400 cursor-not-allowed" : "bg-yellow-500 hover:bg-yellow-700"}`}
                  >
                    Edit
                  </button>
                  <button
                    onClick={() => handleDeletePreset(preset.name)}
                    disabled={preset.name === "Default"}
                    className={`font-bold py-1 px-3 rounded text-sm text-white ${preset.name === "Default" ? "bg-gray-400 cursor-not-allowed" : "bg-red-500 hover:bg-red-700"}`}
                  >
                    Delete
                  </button>
                </div>
              </li>
            ))}
          </ul>
        )}
      </div>
    </>
  );
};

export default PresetsPanel;
