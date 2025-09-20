import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  env: {
    NEXT_PUBLIC_API_BASE_URL: process.env.NEXT_PUBLIC_API_BASE_URL || "http://localhost:8000",
  },
  async rewrites() {
    return [
      {
        source: '/webviz/:path*',
        destination: 'http://localhost:8080/:path*',
      },
    ]
  },
};

export default nextConfig;
