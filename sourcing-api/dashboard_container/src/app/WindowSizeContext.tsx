"use client";

import { useState, useEffect, createContext, useContext } from 'react';

// Define the shape of the context data
interface WindowSize {
  width: number;
  height: number;
}

// Create the context with a default value
const WindowSizeContext = createContext<WindowSize>({ width: 0, height: 0 });

// Custom hook to use the context
export const useWindowSize = () => useContext(WindowSizeContext);

// Provider component
export const WindowSizeProvider = ({ children }: { children: React.ReactNode }) => {
  // Initialize state with server-side rendering safety
  const [windowSize, setWindowSize] = useState<WindowSize>({
    width: typeof window !== 'undefined' ? window.innerWidth : 0,
    height: typeof window !== 'undefined' ? window.innerHeight : 0,
  });

  useEffect(() => {
    // Handler to call on window resize
    const handleResize = () => {
      // Set window width/height in state
      setWindowSize({
        width: window.innerWidth,
        height: window.innerHeight,
      });
    };

    // Add event listener
    window.addEventListener('resize', handleResize);

    // Call handler right away so state gets updated with initial window size
    handleResize();

    // Remove event listener on cleanup
    return () => window.removeEventListener('resize', handleResize);
  }, []); // Empty array ensures that effect is only run on mount and unmount

  return (
    <WindowSizeContext.Provider value={windowSize}>
      {children}
    </WindowSizeContext.Provider>
  );
};
