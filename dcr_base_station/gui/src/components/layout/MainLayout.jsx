'use client';

import React, { useEffect, useRef, useState } from 'react';
import { Box, IconButton } from '@mui/material';
import { ChevronLeft, ChevronRight } from '@mui/icons-material';
import Header from './Header';

const MainLayout = ({ cameraPanel, controlPanel }) => {
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const containerRef = useRef(null);

  // Give the page document focus on mount so the browser Gamepad API fires
  // gamepadconnected as soon as the user presses a button, without requiring
  // a prior mouse click.
  useEffect(() => {
    containerRef.current?.focus();
  }, []);

  return (
    <Box
      ref={containerRef}
      tabIndex={-1}
      sx={{
        height: '100vh',
        backgroundColor: 'background.default',
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden',
        outline: 'none',
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

        {/* Sidebar — collapsible */}
        <Box
          sx={{
            width: sidebarOpen ? 260 : 36,
            flexShrink: 0,
            display: 'flex',
            flexDirection: 'column',
            transition: 'width 0.2s ease',
            overflow: 'hidden',
          }}
        >
          <IconButton
            onClick={() => setSidebarOpen((o) => !o)}
            size="small"
            sx={{ alignSelf: 'flex-start', color: 'text.secondary' }}
          >
            {sidebarOpen ? <ChevronRight /> : <ChevronLeft />}
          </IconButton>

          {sidebarOpen && (
            <Box sx={{ flex: 1, overflowY: 'auto' }}>
              {controlPanel}
            </Box>
          )}
        </Box>
      </Box>
    </Box>
  );
};

export default MainLayout;
