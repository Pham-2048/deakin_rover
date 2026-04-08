'use client';

import { useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';
import { useROSConnection } from './useROSConnection';
import { TOPICS, MSG_TYPES } from '@/lib/utils/constants';

const DEADZONE = 0.1;
const POLL_RATE_HZ = 50;

function applyDeadzone(value, deadzone) {
  if (Math.abs(value) < deadzone) return 0.0;
  return (value - Math.sign(value) * deadzone) / (1.0 - deadzone);
}

/**
 * useGamepad — polls the browser Gamepad API at 50 Hz and publishes
 * sensor_msgs/Joy to /joy via rosbridge whenever ROS is connected.
 *
 * Publishes all raw axes (deadzone-filtered) and buttons.
 * dcr_joy_to_motor on the rover applies its own axis mapping.
 *
 * @returns {{ connected: boolean, gamepadName: string|null }}
 */
export function useGamepad() {
  const { ros, isConnected } = useROSConnection();
  const [gamepadConnected, setGamepadConnected] = useState(false);
  const [gamepadName, setGamepadName] = useState(null);

  const joyTopicRef = useRef(null);
  const rafRef = useRef(null);
  const lastPublishTime = useRef(0);
  const publishIntervalMs = 1000 / POLL_RATE_HZ;

  // Create/destroy the ROSLIB.Topic when ROS connection changes
  useEffect(() => {
    if (isConnected && ros) {
      joyTopicRef.current = new Topic({
        ros,
        name: TOPICS.JOY,
        messageType: MSG_TYPES.JOY,
      });
    } else {
      joyTopicRef.current = null;
    }
  }, [ros, isConnected]);

  // Gamepad connect/disconnect listeners
  useEffect(() => {
    const onConnect = (event) => {
      setGamepadConnected(true);
      setGamepadName(event.gamepad.id);
    };
    const onDisconnect = () => {
      setGamepadConnected(false);
      setGamepadName(null);
      // Publish zero joy when gamepad disconnects to stop rover
      if (joyTopicRef.current) {
        joyTopicRef.current.publish({
          header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
          axes: [],
          buttons: [],
        });
      }
    };

    const checkExisting = () => {
      const existing = navigator.getGamepads ? navigator.getGamepads() : [];
      for (const gp of existing) {
        if (gp) { setGamepadConnected(true); setGamepadName(gp.id); return; }
      }
    };

    window.addEventListener('gamepadconnected', onConnect);
    window.addEventListener('gamepaddisconnected', onDisconnect);
    // Re-check when the tab regains focus (e.g. user switches back from another tab)
    window.addEventListener('focus', checkExisting);

    // Check if a gamepad is already connected (e.g. page refreshed while connected)
    checkExisting();

    return () => {
      window.removeEventListener('gamepadconnected', onConnect);
      window.removeEventListener('gamepaddisconnected', onDisconnect);
      window.removeEventListener('focus', checkExisting);
    };
  }, []);

  // Polling loop
  useEffect(() => {
    const poll = (timestamp) => {
      rafRef.current = requestAnimationFrame(poll);

      if (timestamp - lastPublishTime.current < publishIntervalMs) return;
      lastPublishTime.current = timestamp;

      if (!isConnected || !joyTopicRef.current) return;

      const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
      let activeGamepad = null;
      for (const gp of gamepads) { if (gp) { activeGamepad = gp; break; } }
      if (!activeGamepad) return;

      const axes = Array.from(activeGamepad.axes).map(v => applyDeadzone(v, DEADZONE));
      const buttons = Array.from(activeGamepad.buttons).map(b => (b.pressed ? 1 : 0));

      joyTopicRef.current.publish({
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
        axes,
        buttons,
      });
    };

    rafRef.current = requestAnimationFrame(poll);
    return () => { if (rafRef.current) cancelAnimationFrame(rafRef.current); };
  }, [isConnected, publishIntervalMs]);

  return { connected: gamepadConnected, gamepadName };
}

export default useGamepad;
