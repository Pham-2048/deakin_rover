'use client';

import React, { useState } from 'react';
import {
  Card,
  CardContent,
  Typography,
  Button,
  Box,
  Divider,
  Slider,
} from '@mui/material';
import { useROSConnection } from '@/hooks/useROSConnection';
import { useROSTopic } from '@/hooks/useROSTopic';
import { TOPICS, MSG_TYPES } from '@/lib/utils/constants';

const EndEffectorControl = () => {
  const { isConnected } = useROSConnection();
  const { publish } = useROSTopic(TOPICS.ANTENNA_TRIGGER, MSG_TYPES.STRING);
  const [eePos, setEePos] = useState(140);

  const handleSend = () => {
    publish({ data: `${eePos}\n` });
  };

  return (
    <Card sx={{ mt: 2 }}>
      <CardContent>
        <Typography variant="h6" gutterBottom sx={{ color: 'primary.main' }}>
          End Effector
        </Typography>

        <Divider sx={{ my: 2, borderColor: 'primary.main', opacity: 0.3 }} />

        <Box sx={{ px: 1 }}>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
            <Typography variant="body2" color="text.secondary">Position</Typography>
            <Typography variant="body2" fontFamily="monospace">{eePos}</Typography>
          </Box>
          <Slider
            value={eePos}
            min={90}
            max={140}
            step={1}
            onChange={(_, v) => setEePos(v)}
            disabled={!isConnected}
            marks={[
              { value: 90, label: '90 (closed)' },
              { value: 140, label: '140 (open)' },
            ]}
            sx={{ mb: 2 }}
          />
          <Button
            variant="contained"
            onClick={handleSend}
            disabled={!isConnected}
            fullWidth
          >
            Send
          </Button>
        </Box>
      </CardContent>
    </Card>
  );
};

export default EndEffectorControl;
