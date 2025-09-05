"use client";

import React, { useEffect, useRef } from 'react';

export default function WebvizTab() {
  const iframeRef = useRef<HTMLIFrameElement>(null);

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

    // Clean up the event listener on component unmount
    return () => {
      window.removeEventListener("message", handleMessage);
    };
  }, []); // Empty dependency array means this effect runs once on mount and cleans up on unmount

  return (
    <div className="h-full w-full flex flex-col items-center justify-center">
      <h2 className="text-2xl font-semibold mb-4">Webviz</h2>
      <iframe
        ref={iframeRef}
        src="http://localhost:8080" // Passe ggf. die URL an, falls Webviz woanders läuft
        title="Webviz"
        className="w-full h-[80vh] border rounded"
        allowFullScreen
      />
      <p className="mt-4 text-gray-500 text-sm">Webviz läuft im Container und ist hier eingebettet.</p>
    </div>
  );
}
