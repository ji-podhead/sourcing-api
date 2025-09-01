"use client";
import Link from "next/link";
import { usePathname } from "next/navigation";

const tabs = [
  { name: "Dashboard", path: "/" },
  { name: "Record", path: "/record" },
  { name: "Replay", path: "/replay" },
  { name: "Performance", path: "/performance" },
  { name: "PTP", path: "/ptp" },
  // { name: "ImagingSource", path: "/imagingsource" },
  // { name: "Webviz", path: "/webviz" },
];

export default function Taskbar() {
  const pathname = usePathname();
  return (
    <nav className="w-full flex justify-center items-center bg-gray-900 text-white p-2 shadow mb-8 bg-gradient-to-r from-gray-800 to-gray-900">

      <div className="flex items-left content-left justify-center min-w-[10%] text-left">
       <div> <h1 className="text-3xl font-sans font-bold bg-gradient-to-r from-blue-500 to-purple-600 bg-clip-text text-transparent">
          Sourcing Api
        </h1>
      </div>
      </div>
      <div className="w-full h-full flex gap-2 justify-start items-center pr-2 ">
      <ul className="flex space-x-8">
        {tabs.map((tab) => (
          <li key={tab.path}>
            <Link
              href={tab.path}
              className={`px-4 py-2 rounded hover:bg-gray-700 transition-colors ${pathname === tab.path ? "bg-gray-700 font-bold" : ""}`}
            >
              {tab.name}
            </Link>
          </li>
        ))}
      </ul>
      </div>
    </nav>
  );
}
