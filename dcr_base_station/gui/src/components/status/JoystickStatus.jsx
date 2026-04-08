'use client';

import React from 'react';
import { SportsEsports as JoystickIcon } from '@mui/icons-material';
import StatusIndicator from './StatusIndicator';
import { useGamepad } from '@/hooks/useGamepad';

const JoystickStatus = () => {
  const { connected } = useGamepad();

  return (
    <StatusIndicator
      label="Joystick"
      active={connected}
      stale={false}
      value={connected ? 'Connected' : 'No Signal'}
      icon={<JoystickIcon fontSize="small" />}
    />
  );
};

export default JoystickStatus;
