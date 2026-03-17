'use client';

import MainLayout from '@/components/layout/MainLayout';
import ConnectionManager from '@/components/control/ConnectionManager';
import CameraFeedPanel from '@/components/camera/CameraFeedPanel';

export default function Home() {
  const cameraPanel = <CameraFeedPanel />;
  const controlPanel = <ConnectionManager />;

  return (
    <MainLayout
      cameraPanel={cameraPanel}
      controlPanel={controlPanel}
    />
  );
}
