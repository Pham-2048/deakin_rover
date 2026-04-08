'use client';

import { useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';
import { useROSConnection } from './useROSConnection';

const DEADZONE = 0.1;
const LINEAR_SCALE = 1.0;   // m/s
const ANGULAR_SCALE = 2.0;  // rad/s
const POLL_RATE_HZ = 50;

function applyDeadzone(value, deadzone) {
  if (Math.abs(value) < deadzone) return 0.0;
  // Re-scale so the output starts at 0 just past the deadzone
  return (value - Math.sign(value) * deadzone) / (1.0 - deadzone);
}

/**
 * useGamepad — polls the browser Gamepad API at 50 Hz and publishes
 * geometry_msgs/Twist to /cmd_vel via rosbridge whenever ROS is connected.
 *
 * Axis mapping (standard gamepad layout):
 *   axes[1]  — left stick Y  (up = negative → negate for forward)
 *   axes[0]  — left stick X  (right = positive → negate for turn-left)
 *
 * @returns {{ connected: boolean, gamepadName: string|null }}
 */
export function useGamepad() {
  const { ros, isConnected } = useROSConnection();
  const [gamepadConnected, setGamepadConnected] = useState(false);
  const [gamepadName, setGamepadName] = useState(null);

  const cmdVelTopicRef = useRef(null);
  const rafRef = useRef(null);
  const lastPublishTime = useRef(0);
  const publishIntervalMs = 1000 / POLL_RATE_HZ;

  // Create/destroy the ROSLIB.Topic when ROS connection changes
  useEffect(() => {
    if (isConnected && ros) {
      cmdVelTopicRef.current = new Topic({
        ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });
    } else {
      cmdVelTopicRef.current = null;
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
      // Publish a zero-velocity stop when gamepad disconnects
      if (cmdVelTopicRef.current) {
        cmdVelTopicRef.current.publish(
          { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }
        );
      }
    };

    window.addEventListener('gamepadconnected', onConnect);
    window.addEventListener('gamepaddisconnected', onDisconnect);

    // Check if a gamepad is already connected (e.g. page refreshed while connected)
    const existing = navigator.getGamepads ? navigator.getGamepads() : [];
    for (const gp of existing) {
      if (gp) {
        setGamepadConnected(true);
        setGamepadName(gp.id);
        break;
      }
    }

    return () => {
      window.removeEventListener('gamepadconnected', onConnect);
      window.removeEventListener('gamepaddisconnected', onDisconnect);
    };
  }, []);

  // Polling loop
  useEffect(() => {
    const poll = (timestamp) => {
      rafRef.current = requestAnimationFrame(poll);

      // Throttle to POLL_RATE_HZ
      if (timestamp - lastPublishTime.current < publishIntervalMs) return;
      lastPublishTime.current = timestamp;

      if (!isConnected || !cmdVelTopicRef.current) return;

      const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
      let activeGamepad = null;
      for (const gp of gamepads) {
        if (gp) { activeGamepad = gp; break; }
      }
      if (!activeGamepad) return;

      // axes[1]: left stick Y — up is negative in browser, negate for forward
      // axes[0]: left stick X — right is positive, negate for counter-clockwise turn
      const rawLinear  = -(activeGamepad.axes[1] ?? 0);
      const rawAngular = -(activeGamepad.axes[0] ?? 0);

      const linear  = applyDeadzone(rawLinear,  DEADZONE) * LINEAR_SCALE;
      const angular = applyDeadzone(rawAngular, DEADZONE) * ANGULAR_SCALE;

      cmdVelTopicRef.current.publish({
        linear:  { x: linear,  y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular },
      });
    };

    rafRef.current = requestAnimationFrame(poll);
    return () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, [isConnected, publishIntervalMs]);

  return { connected: gamepadConnected, gamepadName };
}

export default useGamepad;
