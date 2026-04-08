'use client';

import React, { useState } from 'react';
import {
  Card,
  CardContent,
  Typography,
  TextField,
  Button,
  Box,
  Divider,
} from '@mui/material';
import { useROSConnection } from '@/hooks/useROSConnection';
import { useROSTopic } from '@/hooks/useROSTopic';
import { TOPICS, MSG_TYPES } from '@/lib/utils/constants';

const AntennaControl = () => {
  const { isConnected } = useROSConnection();
  const { publish } = useROSTopic(TOPICS.ANTENNA_TRIGGER, MSG_TYPES.STRING);
  const [letter, setLetter] = useState('');

  const handleSend = () => {
    if (letter) {
      publish({ data: letter + '\n' });
    }
  };

  return (
    <Card sx={{ mt: 2 }}>
      <CardContent>
        <Typography variant="h6" gutterBottom sx={{ color: 'primary.main' }}>
          Antenna Control
        </Typography>

        <Divider sx={{ my: 2, borderColor: 'primary.main', opacity: 0.3 }} />

        <Box sx={{ display: 'flex', gap: 1, alignItems: 'flex-start' }}>
          <TextField
            label="Command"
            value={letter}
            onChange={(e) => setLetter(e.target.value.slice(-1))}
            disabled={!isConnected}
            size="small"
            placeholder="X"
            helperText="Single character"
            inputProps={{ maxLength: 1 }}
            sx={{ width: 110 }}
          />
          <Button
            variant="contained"
            onClick={handleSend}
            disabled={!isConnected || !letter}
            sx={{ mt: 0.25 }}
          >
            Send
          </Button>
        </Box>
      </CardContent>
    </Card>
  );
};

export default AntennaControl;
