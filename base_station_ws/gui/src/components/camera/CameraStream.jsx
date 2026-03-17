'use client';

import React, { useState, useCallback } from 'react';
import { Box, Typography, IconButton } from '@mui/material';
import {
  VideocamOff as DisconnectedIcon,
  Refresh as RetryIcon,
  BrokenImage as ErrorIcon,
} from '@mui/icons-material';
import { useCameraStream } from '@/hooks/useCameraStream';

/**
 * Single camera feed component. Renders an MJPEG stream from web_video_server.
 *
 * @param {Object} props
 * @param {string} props.topic - ROS image topic
 * @param {string} props.label - Display label for this camera
 * @param {Object} [props.streamOptions] - Quality/resolution options
 */
const CameraStream = ({ topic, label, streamOptions = {} }) => {
  const { streamUrl, isAvailable, handleLoad, handleError, streamError, isLoading, retry } =
    useCameraStream(topic, streamOptions);

  // Track key to force remount of img element on retry
  const [retryKey, setRetryKey] = useState(0);

  const handleRetry = useCallback(() => {
    retry();
    setRetryKey((k) => k + 1);
  }, [retry]);

  // Disconnected / no topic state
  if (!isAvailable) {
    return (
      <Box
        sx={{
          width: '100%',
          height: '100%',
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          backgroundColor: '#0a0e27',
          border: '1px solid #00d9ff22',
          borderRadius: 1,
          minHeight: 0,
        }}
      >
        <DisconnectedIcon sx={{ fontSize: 40, color: 'text.disabled', mb: 1 }} />
        <Typography variant="body2" sx={{ color: 'text.disabled', fontSize: '0.75rem' }}>
          {label || 'No Camera'}
        </Typography>
        <Typography variant="caption" sx={{ color: 'text.disabled', fontSize: '0.65rem' }}>
          {topic ? 'Not connected' : 'No topic selected'}
        </Typography>
      </Box>
    );
  }

  // Error state
  if (streamError) {
    return (
      <Box
        sx={{
          width: '100%',
          height: '100%',
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          backgroundColor: '#0a0e27',
          border: '1px solid #ff005533',
          borderRadius: 1,
          minHeight: 0,
        }}
      >
        <ErrorIcon sx={{ fontSize: 40, color: 'error.main', mb: 1 }} />
        <Typography variant="body2" sx={{ color: 'error.main', fontSize: '0.75rem', mb: 1 }}>
          {label || 'Stream Error'}
        </Typography>
        <IconButton onClick={handleRetry} size="small" sx={{ color: 'primary.main' }}>
          <RetryIcon />
        </IconButton>
      </Box>
    );
  }

  return (
    <Box
      sx={{
        width: '100%',
        height: '100%',
        position: 'relative',
        backgroundColor: '#0a0e27',
        border: '1px solid #00d9ff22',
        borderRadius: 1,
        overflow: 'hidden',
        minHeight: 0,
      }}
    >
      {/* Loading overlay */}
      {isLoading && (
        <Box
          sx={{
            position: 'absolute',
            inset: 0,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            backgroundColor: '#0a0e27',
            zIndex: 1,
          }}
        >
          <Typography variant="body2" sx={{ color: 'text.disabled', fontSize: '0.75rem' }}>
            Loading {label || 'stream'}...
          </Typography>
        </Box>
      )}

      {/* MJPEG stream */}
      {/* eslint-disable-next-line @next/next/no-img-element */}
      <img
        key={retryKey}
        src={streamUrl}
        alt={label || `Camera: ${topic}`}
        onLoad={handleLoad}
        onError={handleError}
        style={{
          width: '100%',
          height: '100%',
          objectFit: 'cover',
          display: 'block',
        }}
      />

      {/* Label overlay */}
      <Box
        sx={{
          position: 'absolute',
          bottom: 0,
          left: 0,
          right: 0,
          py: 0.5,
          px: 1,
          background: 'linear-gradient(transparent, rgba(0,0,0,0.7))',
        }}
      >
        <Typography
          variant="caption"
          sx={{ color: '#00d9ff', fontSize: '0.65rem', fontWeight: 600, letterSpacing: '0.05em' }}
        >
          {label}
        </Typography>
      </Box>
    </Box>
  );
};

export default CameraStream;
