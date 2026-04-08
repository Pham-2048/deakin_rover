'use client';

import { useROS } from '@/context/ROSContext';

/**
 * Hook for accessing ROS connection state and methods
 * This is a convenience wrapper around the ROSContext
 *
 * @returns {Object} ROS connection state and methods
 */
export const useROSConnection = () => {
  const context = useROS();

  return {
    // Connection state
    ros: context.ros,
    isConnected: context.isConnected,
    isConnecting: context.isConnecting,
    roverIP: context.roverIP,
    roverPort: context.roverPort,
    videoPort: context.videoPort,
    connectionError: context.connectionError,

    // Methods
    connect: context.connect,
    disconnect: context.disconnect,
    reconnect: context.reconnect,
    updateRoverIP: context.updateRoverIP,
    updateRoverPort: context.updateRoverPort,
    updateVideoPort: context.updateVideoPort,
  };
};

export default useROSConnection;
