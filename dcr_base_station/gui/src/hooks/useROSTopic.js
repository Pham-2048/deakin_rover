'use client';

import { useState, useEffect, useRef, useCallback } from 'react';
import { useROSConnection } from './useROSConnection';
import { subscribeTopic, publishMessage } from '@/lib/ros/topicSubscriber';

/**
 * Hook to subscribe to a ROS topic and receive real-time messages.
 *
 * @param {string} topicName - ROS topic name
 * @param {string} messageType - ROS message type
 * @param {Object} options
 * @param {boolean} options.enabled - Whether to subscribe (default: true)
 * @returns {{ message: Object|null, isSubscribed: boolean, publish: Function }}
 */
export function useROSTopic(topicName, messageType, { enabled = true } = {}) {
  const { ros, isConnected } = useROSConnection();
  const [message, setMessage] = useState(null);
  const subscriptionRef = useRef(null);

  useEffect(() => {
    if (!ros || !isConnected || !enabled || !topicName) return;

    const sub = subscribeTopic(ros, topicName, messageType, (msg) => {
      setMessage(msg);
    });

    subscriptionRef.current = sub;

    return () => {
      if (subscriptionRef.current) {
        subscriptionRef.current.unsubscribe();
        subscriptionRef.current = null;
      }
      setMessage(null);
    };
  }, [ros, isConnected, topicName, messageType, enabled]);

  const publish = useCallback(
    (data) => {
      if (ros && isConnected) {
        publishMessage(ros, topicName, messageType, data);
      }
    },
    [ros, isConnected, topicName, messageType]
  );

  const isSubscribed = !!(ros && isConnected && enabled && topicName);

  return { message, isSubscribed, publish };
}

export default useROSTopic;
