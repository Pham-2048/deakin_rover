'use client';

import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import { useROSConnection } from '@/hooks/useROSConnection';
import { subscribeTopic } from '@/lib/ros/topicSubscriber';
import { TOPICS, MSG_TYPES } from '@/lib/utils/constants';

const SystemStatusContext = createContext(null);

const INITIAL_STATUS = {
  can: { connected: false, lastUpdate: null },
  rs485: { connected: false, lastUpdate: null },
  power: { voltage: 0, current: 0, percentage: 0, lastUpdate: null },
  joystick: { connected: false, axes: [], buttons: [], lastUpdate: null },
  network: { ssid: '', signalStrength: 0, latency: 0, lastUpdate: null },
  nodes: [],
};

export const SystemStatusProvider = ({ children }) => {
  const { ros, isConnected } = useROSConnection();
  const [status, setStatus] = useState(INITIAL_STATUS);

  // Subscribe to all status topics when connected
  useEffect(() => {
    if (!ros || !isConnected) return;

    const subscriptions = [];

    // CAN Bus Status
    subscriptions.push(
      subscribeTopic(ros, TOPICS.CAN_STATUS, MSG_TYPES.BOOL, (msg) => {
        setStatus((prev) => ({
          ...prev,
          can: { connected: msg.data, lastUpdate: Date.now() },
        }));
      })
    );

    // RS485 Status
    subscriptions.push(
      subscribeTopic(ros, TOPICS.RS485_STATUS, MSG_TYPES.BOOL, (msg) => {
        setStatus((prev) => ({
          ...prev,
          rs485: { connected: msg.data, lastUpdate: Date.now() },
        }));
      })
    );

    // Power / Battery Status
    subscriptions.push(
      subscribeTopic(ros, TOPICS.POWER, MSG_TYPES.BATTERY_STATE, (msg) => {
        setStatus((prev) => ({
          ...prev,
          power: {
            voltage: msg.voltage || 0,
            current: msg.current || 0,
            percentage: msg.percentage || 0,
            lastUpdate: Date.now(),
          },
        }));
      })
    );

    // Joystick
    subscriptions.push(
      subscribeTopic(ros, TOPICS.JOYSTICK, MSG_TYPES.JOY, (msg) => {
        setStatus((prev) => ({
          ...prev,
          joystick: {
            connected: true,
            axes: msg.axes || [],
            buttons: msg.buttons || [],
            lastUpdate: Date.now(),
          },
        }));
      })
    );

    // Network Status (JSON-encoded string)
    subscriptions.push(
      subscribeTopic(ros, TOPICS.NETWORK, MSG_TYPES.NETWORK_STATUS, (msg) => {
        try {
          const data = JSON.parse(msg.data);
          setStatus((prev) => ({
            ...prev,
            network: {
              ssid: data.ssid || '',
              signalStrength: data.signal_strength || 0,
              latency: data.latency || 0,
              lastUpdate: Date.now(),
            },
          }));
        } catch {
          // If not JSON, treat as plain SSID
          setStatus((prev) => ({
            ...prev,
            network: { ...prev.network, ssid: msg.data, lastUpdate: Date.now() },
          }));
        }
      })
    );

    // Node list (JSON-encoded string)
    subscriptions.push(
      subscribeTopic(ros, TOPICS.NODES, MSG_TYPES.NODE_LIST, (msg) => {
        try {
          const nodes = JSON.parse(msg.data);
          setStatus((prev) => ({
            ...prev,
            nodes: Array.isArray(nodes) ? nodes : [],
          }));
        } catch {
          // ignore parse errors
        }
      })
    );

    return () => {
      subscriptions.forEach((sub) => sub.unsubscribe());
      setStatus(INITIAL_STATUS);
    };
  }, [ros, isConnected]);

  // Detect stale joystick (no message in 2 seconds)
  useEffect(() => {
    if (!isConnected) return;

    const interval = setInterval(() => {
      setStatus((prev) => {
        const now = Date.now();
        const joyStale = prev.joystick.lastUpdate && now - prev.joystick.lastUpdate > 2000;
        if (joyStale && prev.joystick.connected) {
          return { ...prev, joystick: { ...prev.joystick, connected: false } };
        }
        return prev;
      });
    }, 2000);

    return () => clearInterval(interval);
  }, [isConnected]);

  const value = { status };

  return (
    <SystemStatusContext.Provider value={value}>
      {children}
    </SystemStatusContext.Provider>
  );
};

export const useSystemStatusContext = () => {
  const context = useContext(SystemStatusContext);
  if (!context) {
    throw new Error('useSystemStatusContext must be used within a SystemStatusProvider');
  }
  return context;
};

export default SystemStatusContext;
