'use client';

import { useEffect, useRef, useState } from 'react';
import { Topic } from 'roslib';
import { useROSConnection } from './useROSConnection';

const DEADZONE = 0.1;
const POLL_RATE_HZ = 50;

function applyDeadzone(value, deadzone) {
  if (Math.abs(value) < deadzone) return 0.0;
  return (value - Math.sign(value) * deadzone) / (1.0 - deadzone);
}

/**
 * useArmGamepad — polls the browser Gamepad API at 50 Hz and publishes
 * sensor_msgs/Joy to /joy via rosbridge whenever ROS is connected.
 *
 * Publishes ALL raw axes and buttons (motor_node applies its own scaling).
 * Per-axis deadzone is applied before publishing.
 *
 * @returns {{ connected: boolean, gamepadName: string|null }}
 */
export function useArmGamepad({ topic = '/joy' } = {}) {
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
        name: topic,
        messageType: 'sensor_msgs/Joy',
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
      // Publish zero joy when gamepad disconnects to stop arm
      if (joyTopicRef.current) {
        joyTopicRef.current.publish({
          header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
          axes: new Array(8).fill(0.0),
          buttons: new Array(12).fill(0),
        });
      }
    };

    const checkExisting = () => {
      const existing = navigator.getGamepads ? navigator.getGamepads() : [];
      for (const gp of existing) {
        if (gp) {
          setGamepadConnected(true);
          setGamepadName(gp.id);
          return;
        }
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
      for (const gp of gamepads) {
        if (gp) { activeGamepad = gp; break; }
      }
      if (!activeGamepad) return;

      const rawAxes = Array.from(activeGamepad.axes).map((v) => applyDeadzone(v, DEADZONE));
      const rawButtons = Array.from(activeGamepad.buttons).map((b) => (b.pressed ? 1 : 0));

      // Remap browser Gamepad API layout → jsdev layout expected by controller.py.
      // Browser axes: [Lx, Ly, Rx, Ry]
      // jsdev axes:   [Lx, Ly, L2, Rx, Ry, R2, DpadX, DpadY]
      // Browser D-pad: buttons[12]=up, [13]=down, [14]=left, [15]=right (not axes)
      const dpadX = (rawButtons[14] ?? 0) - (rawButtons[15] ?? 0); // left=+1, right=-1
      const dpadY = (rawButtons[13] ?? 0) - (rawButtons[12] ?? 0); // up=-1, down=+1

      const axes = [
        -(rawAxes[0] ?? 0.0),  // [0] Lx      — motor 1 / IK x  (left=+1)
        -(rawAxes[1] ?? 0.0),  // [1] Ly      — motor 2 / IK y  (up=+1)
        0.0,                   // [2] L2 analog (unused)
        -(rawAxes[2] ?? 0.0),  // [3] Rx      — motor 4          (left=+1)
        -(rawAxes[3] ?? 0.0),  // [4] Ry      — motor 3 / IK z  (up=+1)
        0.0,                   // [5] R2 analog (unused)
        dpadX,                 // [6] D-pad X — motor 6          (left=+1)
        dpadY,                 // [7] D-pad Y — motor 5          (up=+1)
      ];

      // Browser buttons[2]=Square, [3]=Triangle — controller.py expects Square at [3].
      // Browser buttons[16]=PS/Home (if exposed), [10]=L3 — controller.py expects PS at [10].
      const buttons = [
        rawButtons[0]  ?? 0,                          // [0]  Cross    — laser
        rawButtons[1]  ?? 0,                          // [1]  Circle   — open EE
        rawButtons[3]  ?? 0,                          // [2]  Triangle (unused)
        rawButtons[2]  ?? 0,                          // [3]  Square   — close EE
        rawButtons[4]  ?? 0,                          // [4]  L1       — mode toggle
        rawButtons[5]  ?? 0,                          // [5]  R1       — mode toggle
        rawButtons[6]  ?? 0,                          // [6]  L2 digital
        rawButtons[7]  ?? 0,                          // [7]  R2 digital
        rawButtons[8]  ?? 0,                          // [8]  Share
        rawButtons[9]  ?? 0,                          // [9]  Options
        rawButtons[16] ?? rawButtons[10] ?? 0,        // [10] PS/Home  — clear faults
        rawButtons[11] ?? 0,                          // [11] R3
      ];

      joyTopicRef.current.publish({
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
        axes,
        buttons,
      });
    };

    rafRef.current = requestAnimationFrame(poll);
    return () => {
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
    };
  }, [isConnected, publishIntervalMs]);

  return { connected: gamepadConnected, gamepadName };
}

export default useArmGamepad;
