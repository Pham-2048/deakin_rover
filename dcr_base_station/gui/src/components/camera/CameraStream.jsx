'use client';

import React, { useState, useCallback } from 'react';
import { Box, Typography, IconButton, Dialog } from '@mui/material';
import {
  VideocamOff as DisconnectedIcon,
  Refresh as RetryIcon,
  BrokenImage as ErrorIcon,
  Fullscreen as FullscreenIcon,
  Close as CloseIcon,
  RotateLeft as RotateLeftIcon,
  RotateRight as RotateRightIcon,
} from '@mui/icons-material';

/**
 * Single camera feed component. Renders an MJPEG stream from mjpg-streamer.
 *
 * @param {Object} props
 * @param {string} props.url            - Full stream URL (e.g. http://host:port/?action=stream)
 * @param {string} props.label          - Display label for this camera
 * @param {number} [props.defaultRotation=180] - Initial rotation in degrees (0/90/180/270)
 */
const CameraStream = ({ url, label, defaultRotation = 180 }) => {
  const [rotation, setRotation] = useState(defaultRotation);
  const [isLoading, setIsLoading] = useState(true);
  const [streamError, setStreamError] = useState(false);
  const [retryKey, setRetryKey] = useState(0);
  const [fullscreenOpen, setFullscreenOpen] = useState(false);

  const handleLoad = useCallback(() => {
    setIsLoading(false);
    setStreamError(false);
  }, []);

  const handleError = useCallback(() => {
    setIsLoading(false);
    setStreamError(true);
  }, []);

  const handleRetry = useCallback(() => {
    setStreamError(false);
    setIsLoading(true);
    setRetryKey((k) => k + 1);
  }, []);

  const rotateCW = useCallback(() => setRotation((r) => (r + 90) % 360), []);
  const rotateCCW = useCallback(() => setRotation((r) => (r - 90 + 360) % 360), []);

  // No URL state
  if (!url) {
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
          No stream URL
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

  const imgStyle = {
    width: '100%',
    height: '100%',
    objectFit: 'cover',
    display: 'block',
    transform: `rotate(${rotation}deg)`,
  };

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
      <img
        key={retryKey}
        src={url}
        alt={label}
        onLoad={handleLoad}
        onError={handleError}
        style={imgStyle}
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

      {/* Rotation buttons — bottom-left */}
      <Box
        sx={{
          position: 'absolute',
          bottom: 4,
          left: 4,
          zIndex: 2,
          display: 'flex',
          gap: 0.5,
        }}
      >
        <IconButton
          size="small"
          onClick={rotateCCW}
          sx={{ color: 'rgba(255,255,255,0.7)', bgcolor: 'rgba(0,0,0,0.35)', p: 0.5, '&:hover': { bgcolor: 'rgba(0,0,0,0.6)', color: '#fff' } }}
        >
          <RotateLeftIcon fontSize="small" />
        </IconButton>
        <IconButton
          size="small"
          onClick={rotateCW}
          sx={{ color: 'rgba(255,255,255,0.7)', bgcolor: 'rgba(0,0,0,0.35)', p: 0.5, '&:hover': { bgcolor: 'rgba(0,0,0,0.6)', color: '#fff' } }}
        >
          <RotateRightIcon fontSize="small" />
        </IconButton>
      </Box>

      {/* Fullscreen button — top-right */}
      <IconButton
        size="small"
        onClick={() => setFullscreenOpen(true)}
        sx={{
          position: 'absolute',
          top: 4,
          right: 4,
          zIndex: 2,
          color: 'rgba(255,255,255,0.7)',
          bgcolor: 'rgba(0,0,0,0.35)',
          '&:hover': { bgcolor: 'rgba(0,0,0,0.6)', color: '#fff' },
          p: 0.5,
        }}
      >
        <FullscreenIcon fontSize="small" />
      </IconButton>

      {/* Fullscreen dialog */}
      <Dialog
        fullScreen
        open={fullscreenOpen}
        onClose={() => setFullscreenOpen(false)}
        PaperProps={{ sx: { bgcolor: 'black' } }}
      >
        <Box sx={{ width: '100%', height: '100%', position: 'relative' }}>
          <IconButton
            onClick={() => setFullscreenOpen(false)}
            sx={{
              position: 'absolute',
              top: 12,
              right: 12,
              zIndex: 10,
              color: 'white',
              bgcolor: 'rgba(0,0,0,0.5)',
              '&:hover': { bgcolor: 'rgba(0,0,0,0.8)' },
            }}
          >
            <CloseIcon />
          </IconButton>
          <img
            src={url}
            alt={label}
            style={{ width: '100%', height: '100%', objectFit: 'contain', display: 'block', transform: `rotate(${rotation}deg)` }}
          />
          <Box
            sx={{
              position: 'absolute',
              bottom: 0,
              left: 0,
              right: 0,
              py: 1,
              px: 2,
              background: 'linear-gradient(transparent, rgba(0,0,0,0.7))',
            }}
          >
            <Typography variant="body2" sx={{ color: '#00d9ff', fontWeight: 600, letterSpacing: '0.05em' }}>
              {label}
            </Typography>
          </Box>
        </Box>
      </Dialog>
    </Box>
  );
};

export default CameraStream;
