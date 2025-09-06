const getApiConfig = () => {
  if (typeof window === 'undefined') {
    // Return a default or placeholder for server-side rendering
    return {
      apiUrl: 'http://localhost:8000',
      wsUrl: 'ws://localhost:8000',
    };
  }

  const protocol = window.location.protocol === 'https:' ? 'https' : 'http';
  const wsProtocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
  const hostname = window.location.hostname;
  const port = '8000'; // The backend port is fixed to 8000

  return {
    apiUrl: `${protocol}://${hostname}:${port}`,
    wsUrl: `${wsProtocol}://${hostname}:${port}`,
  };
};

const apiConfig = getApiConfig();

export const ROS_API_URL = apiConfig.apiUrl;
export const LOGS_WEBSOCKET_URL = `${apiConfig.wsUrl}/logs`;
export const TERMINAL_WEBSOCKET_URL = `${apiConfig.wsUrl}/terminal`;
