'use client';

import React from 'react';
import { Box, Typography } from '@mui/material';

/**
 * Animated pulsing status badge with label.
 *
 * @param {Object} props
 * @param {boolean} props.active - Whether the status is active/connected
 * @param {string} props.label - Status label text
 * @param {string} props.activeColor - Color when active (default: success green)
 * @param {string} props.inactiveColor - Color when inactive (default: error red)
 */
const StatusBadge = ({
  active,
  label,
  activeColor = '#00ff88',
  inactiveColor = '#ff0055',
}) => {
  const color = active ? activeColor : inactiveColor;

  return (
    <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
      <Box
        sx={{
          width: 10,
          height: 10,
          borderRadius: '50%',
          backgroundColor: color,
          boxShadow: `0 0 8px ${color}, 0 0 16px ${color}66`,
          animation: active ? 'statusPulse 2s ease-in-out infinite' : 'none',
          '@keyframes statusPulse': {
            '0%, 100%': { boxShadow: `0 0 8px ${color}, 0 0 16px ${color}66` },
            '50%': { boxShadow: `0 0 12px ${color}, 0 0 24px ${color}aa` },
          },
        }}
      />
      <Typography variant="body2" sx={{ color: 'text.secondary' }}>
        {label}
      </Typography>
    </Box>
  );
};

export default StatusBadge;
