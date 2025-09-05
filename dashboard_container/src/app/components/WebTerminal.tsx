"use client";

import React, { useEffect, useRef } from 'react';
import '@xterm/xterm/css/xterm.css'; // Import xterm's CSS

interface WebTerminalProps {
  websocketUrl: string;
}

const isWebGl2Supported = typeof window !== 'undefined' && !!document.createElement('canvas').getContext('webgl2');

const WebTerminal: React.FC<WebTerminalProps> = ({ websocketUrl }) => {
  const terminalRef = useRef<HTMLDivElement>(null);
  const xtermRef = useRef<any | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const fitAddonRef = useRef<any | null>(null); // Keep FitAddon for resizing

  useEffect(() => {
    const initTerminal = async () => {
      if (terminalRef.current) {
        const { Terminal } = await import('@xterm/xterm');
        const { CanvasAddon } = await import('@xterm/addon-canvas');
        const { WebglAddon } = await import( '@xterm/addon-webgl');
        const { WebLinksAddon } = await import('@xterm/addon-web-links');
        const FontFaceObserver = (await import('fontfaceobserver')).default;
        const { FitAddon } = await import('@xterm/addon-fit'); // Import FitAddon here

        const terminal = new Terminal({
          convertEol: true,
          fontFamily: `'Fira Mono', monospace`,
          fontSize: 14,
          theme: {
            background: '#1e1e1e',
            foreground: '#cccccc',
            cursor: '#cccccc',
            selectionBackground: '#5f5f5f',
          },
          allowProposedApi: true, // Required for some addons
        });
        xtermRef.current = terminal;

        // Load other addons before terminal.open()
        terminal.loadAddon(new WebLinksAddon());

        // Removed conditional loading of WebglAddon/CanvasAddon and Unicode11Addon
        // to simplify and see if the issue persists without them.

        const fitAddon = new FitAddon(); // Initialize FitAddon
        fitAddonRef.current = fitAddon;
        terminal.loadAddon(fitAddon);

        const waitForWebFontAsync = async (termInstance: any) => {
          const fontFamily = termInstance.options.fontFamily;
          if (!fontFamily) return;
          const regular = new FontFaceObserver(fontFamily).load();
          try {
            await regular;
          } catch {
            termInstance.options.fontFamily = "monospace";
          }
        };

        waitForWebFontAsync(terminal).then(() => {
          if (xtermRef.current !== terminal) {
            return;
          }
          terminal.open(terminalRef.current!);
          fitAddon.fit(); // Fit after opening and font loading
        });

        // Connect to WebSocket
        wsRef.current = new WebSocket(websocketUrl);

        wsRef.current.onopen = () => {
          console.log('WebSocket connected for terminal');
          terminal.write('\x1b[32mConnected to backend terminal.\x1b[0m\r\n');
        };

        wsRef.current.onmessage = (event) => {
          console.log('Received message:', event.data); // Log the received data
          if (typeof event.data === 'string') {
            terminal.write(event.data);
          } else if (event.data instanceof Blob) {
            // If it's a Blob, try to read it as text
            const reader = new FileReader();
            reader.onload = () => {
              if (typeof reader.result === 'string') {
                terminal.write(reader.result);
              } else {
                console.error('Failed to read Blob as string');
                terminal.write('\x1b[31mError: Could not read WebSocket message as text.\x1b[0m\r\n');
              }
            };
            reader.onerror = () => {
              console.error('Error reading Blob:', reader.error);
              terminal.write('\x1b[31mError: Failed to read WebSocket message.\x1b[0m\r\n');
            };
            reader.readAsText(event.data);
          } else {
            console.error('Unexpected message data type:', typeof event.data, event.data);
            terminal.write('\x1b[31mError: Unexpected WebSocket message format.\x1b[0m\r\n');
          }
        };

        wsRef.current.onerror = (error) => {
          console.error('WebSocket error:', error);
          terminal.write(`\x1b[31mWebSocket error: ${error}\x1b[0m\r\n`);
        };

        wsRef.current.onclose = () => {
          console.log('WebSocket disconnected for terminal');
          terminal.write('\x1b[31mDisconnected from backend terminal.\x1b[0m\r\n');
        };

        // Handle user input
        terminal.onData((data) => {
          if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
            wsRef.current.send(data);
          }
        });

        // Handle window resize
        const handleResize = () => {
          fitAddon.fit();
        };
        window.addEventListener('resize', handleResize);

        return () => {
          window.removeEventListener('resize', handleResize);
          terminal.dispose();
          if (wsRef.current) {
            wsRef.current.close();
          }
        };
      }
    };

    initTerminal();
  }, [websocketUrl]);

  return (
    <div ref={terminalRef} style={{ width: '100%', height: '400px', backgroundColor: '#1e1e1e' }} />
  );
};

export default WebTerminal;
