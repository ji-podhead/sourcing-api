import DeviceSubTaskbar from "../components/DeviceSubTaskbar";

const subTabs = [
  { name: "Driver Control", path: "/driver" },
  { name: "Configuration", path: "/config" },
];

export default function ImagingSourceLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <div className="flex flex-col items-center w-full">
      <h1 className="text-3xl font-bold mb-6">ImagingSource Device</h1>
      <DeviceSubTaskbar basePath="/imagingsource" tabs={subTabs} />
      <div className="w-full max-w-4xl">
        {children}
      </div>
    </div>
  );
}
