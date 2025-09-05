// Shared types for dashboard and device components

export interface DeviceStatus {
  id: string;
  identifier: string;
  type: string;
  config: any;
  status: string;
  features?: CameraFeatureGroup[];
  name: string;
  path: string;
  apiEndpoint: string;
  fileName: string;
  user_notes?: string;
  publishing_preset?: string;
}

export interface CameraFeatureGroup {
  name: string;
  features: CameraFeature[];
}

export interface CameraFeature {
  name: string;
  description: string;
  type: string;
  value: any;
  min?: number;
  max?: number;
  options?: string[];
  is_writable?: boolean;
}

export type StaticDeviceInfo = {
  name: string;
  path: string;
  apiEndpoint: string;
  fileName: string;
};

export interface Preset {
  name: string;
  configuration: Record<string, Record<string, { type: string; value: any }>>;
}
