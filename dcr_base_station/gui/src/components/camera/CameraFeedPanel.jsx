'use client';

import React, { useState, useCallback } from 'react';
import { Box } from '@mui/material';
import GlowCard from '@/components/common/GlowCard';
import MultiCameraViewer from './MultiCameraViewer';
import { DEFAULT_STREAM_URLS } from '@/lib/utils/constants';

/**
 * Main camera panel. Contains the 2x2 camera grid with per-slot URL inputs.
 */
const CameraFeedPanel = () => {
  // Only front and back cameras on the main drive page
  const [cameras, setCameras] = useState(() =>
    DEFAULT_STREAM_URLS.slice(0, 2).map((cam) => ({ url: cam.url, label: cam.label }))
  );

  const handleCameraChange = useCallback((index, newUrl) => {
    setCameras((prev) => {
      const next = [...prev];
      next[index] = { url: newUrl, label: newUrl || 'Empty' };
      return next;
    });
  }, []);

  return (
    <GlowCard
      title="Camera Feeds"
      sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}
    >
      <Box sx={{ flex: 1, minHeight: 0 }}>
        <MultiCameraViewer
          cameras={cameras}
          onCameraChange={handleCameraChange}
        />
      </Box>
    </GlowCard>
  );
};

export default CameraFeedPanel;
