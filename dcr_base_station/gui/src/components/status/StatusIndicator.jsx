'use client';

import React from 'react';
import { Box, Typography } from '@mui/material';

/**
 * Reusable status indicator row with glowing dot, label, and optional value.
 *
 * @param {Object} props
 * @param {string} props.label - Label text (e.g., "CAN Bus")
 * @param {boolean} props.active - Green (true) or red (false)
 * @param {string} [props.value] - Optional value text (e.g., "24.3V")
 * @param {React.ReactNode} [props.icon] - Optional MUI icon
 * @param {boolean} [props.stale] - Data hasn't been received yet (grey dot)
 */
const StatusIndicator = ({ label, active, value, icon, stale = false }) => {
  const color = stale ? '#6a7a9a' : active ? '#00ff88' : '#ff0055';

  return (
    <Box
      sx={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        py: 0.75,
        px: 1,
        borderRadius: 1,
        backgroundColor: 'background.default',
        border: '1px solid',
        borderColor: `${color}33`,
        transition: 'all 0.3s ease',
      }}
    >
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1.5 }}>
        {/* Glowing status dot */}
        <Box
          sx={{
            width: 10,
            height: 10,
            borderRadius: '50%',
            backgroundColor: color,
            boxShadow: `0 0 6px ${color}, 0 0 12px ${color}66`,
            animation: !stale && active ? 'indicatorPulse 2s ease-in-out infinite' : 'none',
            '@keyframes indicatorPulse': {
              '0%, 100%': { opacity: 1 },
              '50%': { opacity: 0.6 },
            },
          }}
        />
        {icon && (
          <Box sx={{ color: 'text.secondary', display: 'flex', fontSize: 18 }}>
            {icon}
          </Box>
        )}
        <Typography variant="body2" sx={{ color: 'text.primary' }}>
          {label}
        </Typography>
      </Box>

      {value !== undefined && (
        <Typography
          variant="body2"
          sx={{
            color: stale ? 'text.disabled' : 'primary.main',
            fontWeight: 600,
          }}
        >
          {value}
        </Typography>
      )}
    </Box>
  );
};

export default StatusIndicator;
