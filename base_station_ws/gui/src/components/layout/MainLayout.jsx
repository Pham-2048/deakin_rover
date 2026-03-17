'use client';

import React from 'react';
import { Box } from '@mui/material';
import Header from './Header';

const MainLayout = ({ cameraPanel, controlPanel }) => {
  return (
    <Box
      sx={{
        height: '100vh',
        backgroundColor: 'background.default',
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden',
      }}
    >
      <Header />

      <Box
        sx={{
          flex: 1,
          display: 'flex',
          overflow: 'hidden',
          p: 1,
          gap: 1,
        }}
      >
        {/* Camera area — grows to fill available space */}
        <Box sx={{ flex: 1, minWidth: 0 }}>
          {cameraPanel}
        </Box>

        {/* Sidebar — fixed 260px */}
        <Box sx={{ width: 260, flexShrink: 0, overflowY: 'auto' }}>
          {controlPanel}
        </Box>
      </Box>
    </Box>
  );
};

export default MainLayout;
