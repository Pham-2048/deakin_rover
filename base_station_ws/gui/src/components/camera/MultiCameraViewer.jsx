'use client';

import React from 'react';
import { Box } from '@mui/material';
import CameraStream from './CameraStream';
import CameraSelector from './CameraSelector';

/**
 * 2x2 grid of camera feeds with per-cell topic selection.
 *
 * @param {Object} props
 * @param {Array<{ topic: string, label: string }>} props.cameras - Array of 4 camera configs
 * @param {Function} props.onCameraChange - Called with (index, newTopic) when user changes a camera
 * @param {Object} [props.streamOptions] - Shared quality/resolution options
 */
const MultiCameraViewer = ({ cameras, onCameraChange, streamOptions = {} }) => {
  return (
    <Box sx={{
      display: 'grid',
      gridTemplateColumns: '1fr 1fr',
      gridTemplateRows: '1fr 1fr',
      gap: 1,
      height: '100%',
    }}>
      {cameras.map((cam, index) => (
        <Box key={index} sx={{
          display: 'flex',
          flexDirection: 'column',
          gap: 0.5,
          minHeight: 0,
          ...(index === 0 && { gridColumn: '1 / span 2' }),
        }}>
          <CameraSelector
            value={cam.topic}
            onChange={(newTopic) => onCameraChange(index, newTopic)}
            label={`Slot ${index + 1}`}
          />
          <Box sx={{ flex: 1, minHeight: 0 }}>
            <CameraStream
              topic={cam.topic}
              label={cam.label}
              streamOptions={streamOptions}
            />
          </Box>
        </Box>
      ))}
    </Box>
  );
};

export default MultiCameraViewer;
