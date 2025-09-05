"use client";

import React, { useEffect, useRef, useState } from 'react';

export default function RecordTab() {
  const iframeRef = useRef<HTMLIFrameElement>(null);
  const [isFullscreen, setIsFullscreen] = useState(false);

  const toggleFullscreen = () => {
    const iframe = iframeRef.current;
    if (iframe) {
      if (!isFullscreen) {
        if (iframe.requestFullscreen) {
          iframe.requestFullscreen();
        } else if (iframe.mozRequestFullScreen) { // Firefox
          iframe.mozRequestFullScreen();
        } else if (iframe.webkitRequestFullscreen) { // Chrome, Safari and Opera
          iframe.webkitRequestFullscreen();
        } else if (iframe.msRequestFullscreen) { // IE/Edge
          iframe.msRequestFullscreen();
        }
      } else {
        if (document.exitFullscreen) {
          document.exitFullscreen();
        } else if (document.mozCancelFullScreen) { // Firefox
          document.mozCancelFullScreen();
        } else if (document.webkitExitFullscreen) { // Chrome, Safari and Opera
          document.webkitExitFullscreen();
        } else if (document.msExitFullscreen) { // IE/Edge
          document.msExitFullscreen();
        }
      }
    }
  };

  useEffect(() => {
    const handleMessage = (event: MessageEvent) => {
      // Check if the message is from the expected origin (your webviz server)
      // and if it contains camera state data.
      // You might need to adjust the origin check and data structure
      // based on how your webviz instance sends messages.
      if (event.origin === "http://localhost:8080" && event.data && event.data.type === "camera-state-change") {
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
      <h2 className="text-2xl font-semibold mb-4">Record</h2>
      <p className="mb-4">Hier kannst du Aufnahmen starten und stoppen.</p>
      {/* Die eigentliche Logik ist im Dashboard-Home implementiert */}

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
          src="http://localhost:8080" // Passe ggf. die URL an, falls Webviz woanders läuft
          title="Webviz"
          className={`w-full border rounded ${isFullscreen ? 'h-full' : 'h-[60vh]'}`} // Adjust height when not fullscreen
          allowFullScreen
        />
        <p className="mt-2 text-gray-500 text-sm">Webviz läuft im Container und ist hier eingebettet.</p>
      </div>
    </div>
  );
}
