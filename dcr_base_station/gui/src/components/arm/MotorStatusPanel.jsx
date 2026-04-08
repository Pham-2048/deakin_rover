'use client';

import React, { useState, useEffect } from 'react';
import { Box, Typography } from '@mui/material';
import GlowCard from '@/components/common/GlowCard';
import { useROSTopic } from '@/hooks/useROSTopic';
import { TOPICS, MSG_TYPES } from '@/lib/utils/constants';

const TILE_SX = {
  flex: 1,
  display: 'flex',
  flexDirection: 'column',
  gap: 0.25,
  p: 0.75,
  borderRadius: 1,
  backgroundColor: 'background.default',
  border: '1px solid #00d9ff22',
  minWidth: 0,
};

function MetricTile({ label, value, unit, color = 'primary.main' }) {
  return (
    <Box sx={TILE_SX}>
      <Typography
        variant="caption"
        sx={{ color: 'text.disabled', letterSpacing: '0.08em', textTransform: 'uppercase', fontSize: '0.6rem' }}
      >
        {label}
      </Typography>
      <Typography
        variant="body2"
        sx={{ fontFamily: 'Roboto Mono, monospace', fontWeight: 700, color, fontSize: '0.85rem', lineHeight: 1.2 }}
      >
        {value}
        {unit && (
          <Typography component="span" sx={{ fontSize: '0.65rem', color: 'text.disabled', ml: 0.5 }}>
            {unit}
          </Typography>
        )}
      </Typography>
    </Box>
  );
}

function MotorCard({ id, stat1, stat2 }) {
  const faultColor = stat2?.fault && stat2.fault !== 'none' ? '#ff0055' : '#00ff88';

  return (
    <GlowCard sx={{ p: 1.25 }}>
      {/* Motor ID header */}
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 0.75 }}>
        <Typography
          variant="caption"
          sx={{ color: 'primary.main', letterSpacing: '0.12em', textTransform: 'uppercase', fontWeight: 700, fontSize: '0.65rem' }}
        >
          Motor {id}
        </Typography>
        <Typography
          variant="caption"
          sx={{
            px: 0.75, py: 0.1, borderRadius: 0.5,
            backgroundColor: '#00d9ff22', border: '1px solid #00d9ff44',
            color: 'primary.main', fontFamily: 'Roboto Mono, monospace',
            fontSize: '0.6rem', letterSpacing: '0.05em',
          }}
        >
          MTR {id}
        </Typography>
      </Box>

      {!stat1 && !stat2 ? (
        <Box sx={{ py: 1, textAlign: 'center' }}>
          <Typography variant="caption" sx={{ color: 'text.disabled', fontSize: '0.65rem' }}>
            Waiting for motor {id}
          </Typography>
        </Box>
      ) : (
        <Box sx={{ display: 'flex', flexDirection: 'column', gap: 0.5 }}>
          {/* Mechanical */}
          {stat1 && (
            <>
              <Box sx={{ display: 'flex', gap: 0.5 }}>
                <MetricTile label="Angle" value={Number(stat1.angle).toFixed(2)} unit="°" />
                <MetricTile label="Speed" value={Number(stat1.speed).toFixed(1)} unit="RPM" />
              </Box>
              <Box sx={{ display: 'flex', gap: 0.5 }}>
                <MetricTile label="Current" value={Number(stat1.current).toFixed(2)} unit="A" />
                <MetricTile
                  label="Temp"
                  value={stat1.temp}
                  unit="°C"
                  color={stat1.temp > 85 ? '#ff0055' : stat1.temp > 70 ? '#ffaa00' : 'primary.main'}
                />
              </Box>
            </>
          )}

          {/* Electrical */}
          {stat2 && (
            <>
              <Box sx={{ display: 'flex', gap: 0.5 }}>
                <MetricTile label="Bus V" value={Number(stat2.busv).toFixed(1)} unit="V" />
                <MetricTile label="Bus A" value={Number(stat2.busc).toFixed(2)} unit="A" />
              </Box>

              <Box sx={{ ...TILE_SX, flex: 'unset' }}>
                <Typography variant="caption" sx={{ color: 'text.disabled', letterSpacing: '0.08em', textTransform: 'uppercase', fontSize: '0.6rem' }}>
                  Mode
                </Typography>
                <Typography variant="body2" sx={{ fontFamily: 'Roboto Mono, monospace', fontWeight: 700, color: 'primary.main', fontSize: '0.85rem' }}>
                  {stat2.mode || '—'}
                </Typography>
              </Box>

              <Box sx={{ ...TILE_SX, flex: 'unset', flexDirection: 'row', alignItems: 'center', gap: 1 }}>
                <Box sx={{ width: 8, height: 8, borderRadius: '50%', backgroundColor: faultColor, boxShadow: `0 0 6px ${faultColor}`, flexShrink: 0 }} />
                <Box>
                  <Typography variant="caption" sx={{ color: 'text.disabled', letterSpacing: '0.08em', textTransform: 'uppercase', fontSize: '0.6rem', display: 'block' }}>
                    Fault
                  </Typography>
                  <Typography variant="body2" sx={{ fontFamily: 'Roboto Mono, monospace', fontWeight: 700, color: faultColor, fontSize: '0.85rem' }}>
                    {stat2.fault || '—'}
                  </Typography>
                </Box>
              </Box>
            </>
          )}
        </Box>
      )}
    </GlowCard>
  );
}

/**
 * MotorStatusPanel — shows live telemetry for all 6 arm motors.
 * Accumulates per-motor messages by ID since the rover publishes one motor at a time.
 */
const MotorStatusPanel = () => {
  const [motors1, setMotors1] = useState({});
  const [motors2, setMotors2] = useState({});

  const { message: stat1 } = useROSTopic(TOPICS.MOTOR_STAT1, MSG_TYPES.MOTOR_STAT1);
  const { message: stat2 } = useROSTopic(TOPICS.MOTOR_STAT2, MSG_TYPES.MOTOR_STAT2);

  useEffect(() => {
    if (stat1?.id != null) {
      setMotors1(prev => ({ ...prev, [stat1.id]: stat1 }));
    }
  }, [stat1]);

  useEffect(() => {
    if (stat2?.id != null) {
      setMotors2(prev => ({ ...prev, [stat2.id]: stat2 }));
    }
  }, [stat2]);

  return (
    <Box sx={{ display: 'flex', flexDirection: 'column', gap: 1 }}>
      {[1, 2, 3, 4, 5, 6].map(id => (
        <MotorCard key={id} id={id} stat1={motors1[id]} stat2={motors2[id]} />
      ))}
    </Box>
  );
};

export default MotorStatusPanel;
