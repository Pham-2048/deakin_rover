'use client';

import { Box } from '@mui/material';
import MainLayout from '@/components/layout/MainLayout';
import ConnectionManager from '@/components/control/ConnectionManager';
import AntennaControl from '@/components/control/AntennaControl';
import CameraFeedPanel from '@/components/camera/CameraFeedPanel';
import JoystickStatus from '@/components/status/JoystickStatus';

export default function Home() {
  const cameraPanel = <CameraFeedPanel />;
  const controlPanel = <><ConnectionManager /><AntennaControl /></>;

  return (
    <MainLayout
      cameraPanel={cameraPanel}
      controlPanel={controlPanel}
    />
  );
}
