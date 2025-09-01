import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "ROS Control Dashboard",
  description: "Dashboard for controlling ROS components and managing recordings.",
};

import Taskbar from "./components/Taskbar";
import { WindowSizeProvider } from './WindowSizeContext'; // Import the provider

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body className="font-sans antialiased bg-gray-100 min-h-screen min-w-screen">
        <Taskbar />
        <main className="w-full mx-auto w-full">
          <WindowSizeProvider> {/* Wrap children with the provider */}
            {children}
          </WindowSizeProvider>
        </main>
      </body>
    </html>
  );
}
