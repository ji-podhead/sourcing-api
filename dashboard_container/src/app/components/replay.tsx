"use client";

import { useState, useEffect, useRef } from 'react';

export default function ReplayTab() {
  const API_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';
  const WEBVIZ_URL = process.env.NEXT_PUBLIC_WEBVIZ_URL || 'http://localhost:8080';

  const [recordings, setRecordings] = useState<string[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [playingBag, setPlayingBag] = useState<string | null>(null);
  const iframeRef = useRef<HTMLIFrameElement>(null);
  const [isFullscreen, setIsFullscreen] = useState(false);

  const fetchRecordings = async () => {
    try {
      setLoading(true);
      const response = await fetch(`${API_URL}/recordings`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      setRecordings(data);
    } catch (error: any) {
      setError(`Failed to fetch recordings: ${error.message}`);
      console.error('Error fetching recordings:', error);
    } finally {
      setLoading(false);
    }
  };

  const playRecording = async (bagName: string) => {
    try {
      setPlayingBag(bagName);
      const response = await fetch(`${API_URL}/recordings/play/${bagName}`, {
        method: 'POST',
      });
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      // Optionally fetch status periodically to update UI
    } catch (error: any) {
      setError(`Failed to play recording ${bagName}: ${error.message}`);
      console.error(`Error playing recording ${bagName}:`, error);
      setPlayingBag(null);
    }
  };

  const stopPlayback = async () => {
    try {
      const response = await fetch(`${API_URL}/recordings/stop_playback`, {
        method: 'POST',
      });
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      setPlayingBag(null);
    } catch (error: any) {
      setError(`Failed to stop playback: ${error.message}`);
      console.error('Error stopping playback:', error);
    }
  };

  const toggleFullscreen = () => {
    const iframe = iframeRef.current;
    if (iframe) {
      if (!isFullscreen) {
        if (iframe.requestFullscreen) {
          iframe.requestFullscreen();
        } else {
          // @ts-ignore
          if (iframe.mozRequestFullScreen) { // Firefox
            // @ts-ignore
            iframe.mozRequestFullScreen();
          } else {
            // @ts-ignore
            if (iframe.webkitRequestFullscreen) { // Chrome, Safari and Opera
              // @ts-ignore
              iframe.webkitRequestFullscreen();
            } else {
              // @ts-ignore
              if (iframe.msRequestFullscreen) { // IE/Edge
                // @ts-ignore
                iframe.msRequestFullscreen();
              }
            }
          }
        }
      } else {
        if (document.exitFullscreen) {
          document.exitFullscreen();
        } else {
          // @ts-ignore
          if (document.mozCancelFullScreen) { // Firefox
            // @ts-ignore
            document.mozCancelFullScreen();
          } else {
            // @ts-ignore
            if (document.webkitExitFullscreen) { // Chrome, Safari and Opera
              // @ts-ignore
              document.webkitExitFullscreen();
            } else {
              // @ts-ignore
              if (document.msExitFullscreen) { // IE/Edge
                // @ts-ignore
                document.msExitFullscreen();
              }
            }
          }
        }
      }
    }
  };

  useEffect(() => {
    fetchRecordings();

    const handleMessage = (event: MessageEvent) => {
      // Check if the message is from the expected origin (your webviz server)
      // and if it contains camera state data.
      // You might need to adjust the origin check and data structure
      // based on how your webviz instance sends messages.
      if (event.origin === WEBVIZ_URL && event.data && event.data.type === "camera-state-change") {
        console.log("Camera state changed:", event.data.payload);
        // Implement your onCameraStateChange logic here
        // For example, update some state or send data to another part of your app
      }
    };

    window.addEventListener("message", handleMessage);

    const handleFullscreenChange = () => {
      setIsFullscreen(!!document.fullscreenElement);
    };

    document.addEventListener('fullscreenchange', handleFullscreenChange);
    document.addEventListener('mozfullscreenchange', handleFullscreenChange);
    document.addEventListener('webkitfullscreenchange', handleFullscreenChange);
    document.addEventListener('msfullscreenchange', handleFullscreenChange);


    // Clean up the event listeners on component unmount
    return () => {
      window.removeEventListener("message", handleMessage);
      document.removeEventListener('fullscreenchange', handleFullscreenChange);
      document.removeEventListener('mozfullscreenchange', handleFullscreenChange);
      document.removeEventListener('webkitfullscreenchange', handleFullscreenChange);
      document.removeEventListener('msfullscreenchange', handleFullscreenChange);
    };
  }, []); // Empty dependency array means this effect runs once on mount and cleans up on unmount


  return (
    <div className="h-full w-full flex flex-col items-center p-4">
      <h2 className="text-2xl font-semibold mb-4">Replay</h2>

      {loading && <p>Loading recordings...</p>}
      {error && <p className="text-red-500">Error: {error}</p>}

      {!loading && !error && recordings.length === 0 && (
        <p>No recordings found in /data.</p>
      )}

      {!loading && !error && recordings.length > 0 && (
        <div className="w-full max-w-md mb-4"> {/* Added margin-bottom */}
          <h3 className="text-xl font-medium mb-2">Available Recordings:</h3>
          <ul>
            {recordings.map((bagName) => (
              <li key={bagName} className="flex justify-between items-center bg-gray-100 p-2 rounded mb-2">
                <span>{bagName}</span>
                <div>
                  {playingBag === bagName ? (
                    <button
                      onClick={stopPlayback}
                      className="ml-2 px-3 py-1 bg-red-500 text-white rounded hover:bg-red-600"
                    >
                      Stop
                    </button>
                  ) : (
                    <button
                      onClick={() => playRecording(bagName)}
                      disabled={playingBag !== null} // Disable if another bag is playing
                      className={`ml-2 px-3 py-1 bg-blue-500 text-white rounded ${playingBag !== null ? 'opacity-50 cursor-not-allowed' : 'hover:bg-blue-600'}`}
                    >
                      Play
                    </button>
                  )}
                </div>
              </li>
            ))}
          </ul>
          {playingBag && (
            <p className="mt-4 text-green-600">Currently playing: {playingBag}</p>
          )}
        </div>
      )}

      {/* Webviz iframe section */}
      <div className={`w-full flex flex-col items-center ${isFullscreen ? 'fixed top-0 left-0 w-screen h-screen z-50 bg-white' : ''}`}>
         <div className="flex justify-between items-center w-full max-w-md mb-2">
            <h3 className="text-xl font-semibold">Webviz</h3>
            <button
              onClick={toggleFullscreen}
              className="px-3 py-1 bg-gray-200 rounded hover:bg-gray-300"
            >
              {isFullscreen ? 'Exit Fullscreen' : 'Fullscreen'}
            </button>
         </div>
        <iframe
          ref={iframeRef}
          src={WEBVIZ_URL}
          title="Webviz"
          className={`w-full border rounded ${isFullscreen ? 'h-full' : 'h-[60vh]'}`} // Adjust height when not fullscreen
          allowFullScreen
        />
        <p className="mt-2 text-gray-500 text-sm">Webviz läuft im Container und ist hier eingebettet.</p>
      </div>
    </div>
  );
}
