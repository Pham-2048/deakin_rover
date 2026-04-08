'use client';

import { useState } from 'react';
import { Box, Typography, TextField } from '@mui/material';
import MainLayout from '@/components/layout/MainLayout';
import ConnectionManager from '@/components/control/ConnectionManager';
import EndEffectorControl from '@/components/control/EndEffectorControl';
import MotorStatusPanel from '@/components/arm/MotorStatusPanel';
import CameraStream from '@/components/camera/CameraStream';
import { useArmGamepad } from '@/hooks/useArmGamepad';
import { useROSConnection } from '@/hooks/useROSConnection';
import { DEFAULT_STREAM_URLS, TOPICS } from '@/lib/utils/constants';

function ArmControlPanel() {
  const { isConnected } = useROSConnection();
  const { connected, gamepadName } = useArmGamepad({ topic: TOPICS.ARM_JOY });

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
      <ConnectionManager />

      {isConnected && (
        <>
          <Box
            sx={{
              p: 1.5,
              borderRadius: 1,
              background: connected ? '#1b3a1b' : '#2a2a2a',
              border: '1px solid',
              borderColor: connected ? 'success.dark' : 'divider',
              fontSize: 12,
              fontFamily: 'monospace',
              color: connected ? 'success.light' : 'text.disabled',
              userSelect: 'none',
            }}
          >
            {connected
              ? `Gamepad: ${gamepadName ?? 'connected'} — publishing /arm/joy`
              : 'No gamepad detected — connect a controller to control the arm'}
          </Box>
          <EndEffectorControl />
        </>
      )}
    </Box>
  );
}

function ArmStatusArea() {
  const { isConnected } = useROSConnection();
  const defaultCam = DEFAULT_STREAM_URLS[2];
  const [streamUrl, setStreamUrl] = useState(defaultCam.url);

  return (
    <Box sx={{ display: 'flex', flexDirection: 'row', height: '100%', gap: 1, p: 1 }}>
      <Box sx={{ flex: 1, minWidth: 0, height: '100%', display: 'flex', flexDirection: 'column', gap: 0.5 }}>
        <TextField
          size="small"
          fullWidth
          placeholder="Stream URL (e.g. http://host:port/?action=stream)"
          value={streamUrl}
          onChange={(e) => setStreamUrl(e.target.value)}
          slotProps={{
            input: { sx: { fontSize: '0.75rem', fontFamily: 'monospace' } },
          }}
        />
        <Box sx={{ flex: 1, minHeight: 0 }}>
          <CameraStream url={streamUrl} label={defaultCam.label} />
        </Box>
      </Box>
      <Box sx={{ width: 260, flexShrink: 0, overflowY: 'auto' }}>
        {isConnected ? (
          <MotorStatusPanel />
        ) : (
          <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'center', height: '100%', color: 'text.disabled' }}>
            <Typography variant="body2">Connect to rover to see arm status</Typography>
          </Box>
        )}
      </Box>
    </Box>
  );
}

export default function ArmPage() {
  return (
    <MainLayout
      cameraPanel={<ArmStatusArea />}
      controlPanel={<ArmControlPanel />}
    />
  );
}
