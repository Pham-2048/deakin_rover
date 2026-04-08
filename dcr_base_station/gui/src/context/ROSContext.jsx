'use client';

import React, { createContext, useContext, useState, useCallback, useEffect } from 'react';
import rosBridge from '@/lib/ros/rosbridge';

const ROSContext = createContext(null);

export const ROSProvider = ({ children }) => {
  const [ros, setRos] = useState(null);
  const [isConnected, setIsConnected] = useState(false);
  const [roverIP, setRoverIP] = useState('rover.local');
  const [roverPort, setRoverPort] = useState('9090');
  const [videoPort, setVideoPort] = useState('8080');
  const [connectionError, setConnectionError] = useState(null);
  const [isConnecting, setIsConnecting] = useState(false);

  // Setup event handlers on mount
  useEffect(() => {
    rosBridge.onConnectionChange = (connected) => {
      setIsConnected(connected);
      setIsConnecting(false);
      if (connected) {
        setConnectionError(null);
        setRos(rosBridge.getConnection());
      } else {
        setRos(null);
      }
    };

    rosBridge.onError = (error) => {
      setConnectionError(error.message || 'Connection failed');
      setIsConnecting(false);
    };

    return () => {
      rosBridge.onConnectionChange = null;
      rosBridge.onError = null;
    };
  }, []);

  /**
   * Connect to the rover
   */
  const connect = useCallback(async (customIP = null, customPort = null) => {
    const targetIP = customIP || roverIP;
    const targetPort = customPort || roverPort;
    const url = `ws://${targetIP}:${targetPort}`;

    try {
      setIsConnecting(true);
      setConnectionError(null);

      const connection = await rosBridge.connect(url);
      setRos(connection);

      return connection;
    } catch (error) {
      console.error('[ROSContext] Connection failed:', error);
      setConnectionError(error.message || 'Failed to connect');
      setIsConnecting(false);
      throw error;
    }
  }, [roverIP, roverPort]);

  /**
   * Disconnect from the rover
   */
  const disconnect = useCallback(() => {
    rosBridge.disconnect();
    setRos(null);
    setIsConnected(false);
    setConnectionError(null);
    setIsConnecting(false);
  }, []);

  /**
   * Reconnect to the rover
   */
  const reconnect = useCallback(() => {
    rosBridge.resetReconnectAttempts();
    disconnect();
    setTimeout(() => {
      connect();
    }, 500);
  }, [connect, disconnect]);

  /**
   * Update rover IP
   */
  const updateRoverIP = useCallback((newIP) => {
    setRoverIP(newIP);
  }, []);

  const updateRoverPort = useCallback((newPort) => {
    setRoverPort(newPort);
  }, []);

  const updateVideoPort = useCallback((newPort) => {
    setVideoPort(newPort);
  }, []);

  const value = {
    ros,
    isConnected,
    isConnecting,
    roverIP,
    roverPort,
    videoPort,
    connectionError,
    connect,
    disconnect,
    reconnect,
    updateRoverIP,
    updateRoverPort,
    updateVideoPort,
  };

  return <ROSContext.Provider value={value}>{children}</ROSContext.Provider>;
};

export const useROS = () => {
  const context = useContext(ROSContext);
  if (!context) {
    throw new Error('useROS must be used within a ROSProvider');
  }
  return context;
};

export default ROSContext;
