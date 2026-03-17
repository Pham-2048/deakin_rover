'use client';

import React, { useState, useCallback } from 'react';
import { Box, Typography } from '@mui/material';
import {
  Videocam as CameraIcon,
} from '@mui/icons-material';
import GlowCard from '@/components/common/GlowCard';
import MultiCameraViewer from './MultiCameraViewer';
import { useROSConnection } from '@/hooks/useROSConnection';
import { DEFAULT_CAMERAS, STREAM_DEFAULTS } from '@/lib/utils/constants';

/**
 * Main camera panel. Contains the 2x2 camera grid, stream quality controls,
 * and video server port configuration.
 */
const CameraFeedPanel = () => {
  const { isConnected, videoPort, updateVideoPort } = useROSConnection();

  // Per-slot camera assignments (initialize from defaults)
  const [cameras, setCameras] = useState(() =>
    DEFAULT_CAMERAS.map((cam) => ({ topic: cam.topic, label: cam.label }))
  );

  // Stream quality settings
  const [quality, setQuality] = useState(STREAM_DEFAULTS.quality);

  const handleCameraChange = useCallback((index, newTopic) => {
    // Find the label for the selected topic
    const match = DEFAULT_CAMERAS.find((c) => c.topic === newTopic);
    setCameras((prev) => {
      const next = [...prev];
      next[index] = { topic: newTopic, label: match ? match.label : newTopic || 'Empty' };
      return next;
    });
  }, []);

  const handleVideoPortChange = (e) => {
    updateVideoPort(e.target.value);
  };

  const streamOptions = { quality };

  return (
    <GlowCard
      title="Camera Feeds"
      sx={{ height: '100%', display: 'flex', flexDirection: 'column' }}
    >
      {!isConnected ? (
        <Box
          sx={{
            flex: 1,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            py: 6,
            opacity: 0.5,
          }}
        >
          <CameraIcon sx={{ fontSize: 48, color: 'text.disabled', mb: 1 }} />
          <Typography variant="body2" sx={{ color: 'text.disabled' }}>
            Connect to rover to view camera feeds
          </Typography>
        </Box>
      ) : (
        <Box sx={{ flex: 1, minHeight: 0 }}>
          <MultiCameraViewer
            cameras={cameras}
            onCameraChange={handleCameraChange}
            streamOptions={streamOptions}
          />
        </Box>
      )}
    </GlowCard>
  );
};

export default CameraFeedPanel;
