"use client";
import Link from "next/link";
import { usePathname } from "next/navigation";

interface SubTab {
  name: string;
  path: string;
}

interface DeviceSubTaskbarProps {
  basePath: string;
  tabs: SubTab[];
}

export default function DeviceSubTaskbar({ basePath, tabs }: DeviceSubTaskbarProps) {
  const pathname = usePathname();
  return (
    <nav className="w-full flex justify-center bg-gray-800 text-white py-2 shadow mb-4">
      <ul className="flex space-x-4">
        {tabs.map((tab) => (
          <li key={tab.path}>
            <Link
              href={`${basePath}${tab.path}`}
              className={`px-4 py-2 rounded hover:bg-gray-600 transition-colors ${pathname === `${basePath}${tab.path}` ? "bg-gray-600 font-bold" : ""}`}
            >
              {tab.name}
            </Link>
          </li>
        ))}
      </ul>
    </nav>
  );
}
