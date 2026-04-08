'use client';

import React from 'react';
import { Box, TextField } from '@mui/material';
import CameraStream from './CameraStream';

/**
 * 2x2 grid of camera feeds with per-cell URL text fields.
 *
 * @param {Object} props
 * @param {Array<{ url: string, label: string }>} props.cameras - Array of camera configs
 * @param {Function} props.onCameraChange - Called with (index, newUrl) when user edits a URL
 */
const MultiCameraViewer = ({ cameras, onCameraChange }) => {
  return (
    <Box sx={{
      display: 'grid',
      gridTemplateColumns: '1fr 1fr',
      gridTemplateRows: '1fr',
      gap: 1,
      height: '100%',
      minHeight: 0,
    }}>
      {cameras.map((cam, index) => (
        <Box key={index} sx={{
          display: 'flex',
          flexDirection: 'column',
          gap: 0.5,
          minHeight: 0,
        }}>
          <TextField
            size="small"
            fullWidth
            placeholder="Stream URL (e.g. http://host:port/?action=stream)"
            value={cam.url}
            onChange={(e) => onCameraChange(index, e.target.value)}
            slotProps={{
              input: { sx: { fontSize: '0.75rem', fontFamily: 'monospace' } },
            }}
          />
          <Box sx={{ flex: 1, minHeight: 0 }}>
            <CameraStream url={cam.url} label={cam.label} />
          </Box>
        </Box>
      ))}
    </Box>
  );
};

export default MultiCameraViewer;
