'use client';

import React from 'react';
import { Card, CardContent, Typography, Divider } from '@mui/material';

/**
 * Styled card with Iron Man glow effect and optional title.
 */
const GlowCard = ({ title, children, sx, ...props }) => {
  return (
    <Card sx={sx} {...props}>
      <CardContent sx={{
        display: 'flex',
        flexDirection: 'column',
        flex: 1,
        minHeight: 0,
        p: 2,
        '&:last-child': { pb: 2 },
      }}>
        {title && (
          <>
            <Typography variant="h6" gutterBottom sx={{ color: 'primary.main' }}>
              {title}
            </Typography>
            <Divider sx={{ mb: 2, borderColor: 'primary.main', opacity: 0.3 }} />
          </>
        )}
        {children}
      </CardContent>
    </Card>
  );
};

export default GlowCard;
