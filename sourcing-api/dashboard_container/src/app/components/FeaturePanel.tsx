import React from 'react';
import { CameraFeatureGroup, CameraFeature } from '../types';

interface FeaturePanelProps {
  selectedCamera: any;
  selectedFeatureGroup: string | null;
  handleFeatureGroupSelect: (groupName: string) => void;
  handleFeatureChange: (
    cameraId: string,
    featureGroupName: string,
    featureName: string,
    newValue: any
  ) => void;
}

const FeaturePanel: React.FC<FeaturePanelProps> = ({
  selectedCamera,
  selectedFeatureGroup,
  handleFeatureGroupSelect,
  handleFeatureChange,
}) => {
  if (!selectedCamera?.features || selectedCamera.features.length === 0) {
    return <p>No features available for this camera.</p>;
  }

  return (
    <div className="space-y-4">
      <div className="flex overflow-x-auto space-x-2 mb-4 pb-2 border-b">
        {selectedCamera.features.map((group: CameraFeatureGroup) => (
          <button
            key={group.name}
            onClick={() => handleFeatureGroupSelect(group.name)}
            className={`px-4 py-2 rounded-md text-sm font-medium whitespace-nowrap ${
              selectedFeatureGroup === group.name
                ? 'bg-blue-600 text-white'
                : 'bg-gray-200 hover:bg-gray-300'
            }`}
          >
            {group.name}
          </button>
        ))}
      </div>

      {selectedFeatureGroup && (
        <div className="space-y-4">
          {selectedCamera.features
            .find((g: CameraFeatureGroup) => g.name === selectedFeatureGroup)
            ?.features.map((feature: CameraFeature) => (
              <div
                key={feature.name}
                className={`border p-3 rounded-md ${
                  !feature.is_writable ? 'bg-gray-100' : ''
                }`}
              >
                <p
                  className={`font-semibold ${
                    !feature.is_writable ? 'line-through' : ''
                  }`}
                >
                  {feature.name}{' '}
                  <span className="font-normal text-gray-500 text-sm">
                    ({feature.type})
                  </span>
                </p>
                <p className="text-sm text-black mb-2">{feature.description}</p>

                {feature.type === 'Enumeration' ? (
                  <select
                    value={feature.value}
                    onChange={e =>
                      selectedCamera &&
                      selectedFeatureGroup &&
                      handleFeatureChange(
                        selectedCamera.identifier,
                        selectedFeatureGroup,
                        feature.name,
                        e.target.value
                      )
                    }
                    className="w-full p-2 border rounded-md"
                    disabled={!feature.is_writable}
                  >
                    {feature.options?.map((option: string) => (
                      <option key={option} value={option}>
                        {option}
                      </option>
                    ))}
                  </select>
                ) : feature.type === 'Boolean' ? (
                  <input
                    type="checkbox"
                    checked={feature.value}
                    onChange={e =>
                      selectedCamera &&
                      selectedFeatureGroup &&
                      handleFeatureChange(
                        selectedCamera.identifier,
                        selectedFeatureGroup,
                        feature.name,
                        e.target.checked
                      )
                    }
                    className="h-4 w-4 text-blue-600 border-gray-300 rounded focus:ring-blue-500"
                    disabled={!feature.is_writable}
                  />
                ) : (
                  <input
                    type={
                      feature.type === 'Integer' || feature.type === 'Float'
                        ? 'number'
                        : 'text'
                    }
                    value={feature.value}
                    onChange={e =>
                      selectedCamera &&
                      selectedFeatureGroup &&
                      handleFeatureChange(
                        selectedCamera.identifier,
                        selectedFeatureGroup,
                        feature.name,
                        e.target.value
                      )
                    }
                    className="w-full p-2 border rounded-md"
                    placeholder={feature.type}
                    disabled={!feature.is_writable}
                    min={feature.min}
                    max={feature.max}
                  />
                )}

                <div className="text-xs text-gray-500 mt-2">
                  {feature.min && <span>Min: {feature.min}</span>}
                  {feature.max && (
                    <span className="ml-2">Max: {feature.max}</span>
                  )}
                </div>
              </div>
            ))}
        </div>
      )}
    </div>
  );
};

export default FeaturePanel;
