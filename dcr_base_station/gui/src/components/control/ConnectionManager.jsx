'use client';

import React, { useState } from 'react';
import {
  Card,
  CardContent,
  Typography,
  TextField,
  Button,
  Box,
  Alert,
  Divider,
} from '@mui/material';
import {
  Power as PowerIcon,
  PowerOff as PowerOffIcon,
  Refresh as RefreshIcon,
} from '@mui/icons-material';
import { useROSConnection } from '@/hooks/useROSConnection';

const ConnectionManager = () => {
  const {
    isConnected,
    isConnecting,
    roverIP,
    roverPort,
    connectionError,
    connect,
    disconnect,
    reconnect,
    updateRoverIP,
    updateRoverPort,
  } = useROSConnection();

  const [localIP, setLocalIP] = useState(roverIP);
  const [localPort, setLocalPort] = useState(roverPort);

  const handleConnect = async () => {
    try {
      await connect(localIP, localPort);
    } catch (error) {
      console.error('Connection failed:', error);
    }
  };

  const handleDisconnect = () => {
    disconnect();
  };

  const handleReconnect = () => {
    reconnect();
  };

  const handleIPChange = (e) => {
    const newIP = e.target.value;
    setLocalIP(newIP);
    updateRoverIP(newIP);
  };

  const handlePortChange = (e) => {
    const newPort = e.target.value;
    setLocalPort(newPort);
    updateRoverPort(newPort);
  };

  return (
    <Card>
      <CardContent>
        <Typography variant="h6" gutterBottom sx={{ color: 'primary.main' }}>
          Connection Manager
        </Typography>

        <Divider sx={{ my: 2, borderColor: 'primary.main', opacity: 0.3 }} />

        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 2 }}>
          {/* IP and Port Inputs */}
          <Box sx={{ display: 'flex', gap: 1 }}>
            <TextField
              label="Rover IP Address"
              value={localIP}
              onChange={handleIPChange}
              disabled={isConnected || isConnecting}
              size="small"
              placeholder="rover.local"
              helperText="Hostname or IP"
              sx={{ flex: 1 }}
            />
            <TextField
              label="Port"
              value={localPort}
              onChange={handlePortChange}
              disabled={isConnected || isConnecting}
              size="small"
              placeholder="9090"
              type="number"
              helperText="WS port"
              sx={{ width: 100 }}
              inputProps={{ min: 1, max: 65535 }}
            />
          </Box>

          {/* Connection Error */}
          {connectionError && (
            <Alert severity="error" sx={{ fontSize: '0.875rem' }}>
              {connectionError}
            </Alert>
          )}

          {/* Connection Status */}
          {isConnected && (
            <Alert severity="success" sx={{ fontSize: '0.875rem' }}>
              Connected to {roverIP}:{roverPort}
            </Alert>
          )}

          {/* Action Buttons */}
          <Box sx={{ display: 'flex', gap: 1, flexDirection: 'column' }}>
            {!isConnected ? (
              <Button
                variant="contained"
                onClick={handleConnect}
                disabled={isConnecting || !localIP}
                startIcon={<PowerIcon />}
                fullWidth
              >
                {isConnecting ? 'Connecting...' : 'Connect'}
              </Button>
            ) : (
              <>
                <Button
                  variant="outlined"
                  onClick={handleDisconnect}
                  startIcon={<PowerOffIcon />}
                  fullWidth
                  color="error"
                >
                  Disconnect
                </Button>
                <Button
                  variant="outlined"
                  onClick={handleReconnect}
                  startIcon={<RefreshIcon />}
                  fullWidth
                >
                  Reconnect
                </Button>
              </>
            )}
          </Box>

          {/* Connection Info */}
          <Box
            sx={{
              mt: 2,
              p: 2,
              backgroundColor: 'background.default',
              borderRadius: 1,
              border: '1px solid',
              borderColor: 'primary.main',
              opacity: 0.7,
            }}
          >
            <Typography variant="body2" sx={{ fontFamily: 'monospace' }}>
              Status: {isConnected ? '🟢 Connected' : '🔴 Disconnected'}
            </Typography>
            <Typography variant="body2" sx={{ fontFamily: 'monospace' }}>
              Endpoint: ws://{roverIP}:{roverPort}
            </Typography>
          </Box>
        </Box>
      </CardContent>
    </Card>
  );
};

export default ConnectionManager;
