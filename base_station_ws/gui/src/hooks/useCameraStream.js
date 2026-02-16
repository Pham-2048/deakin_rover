'use client';

import { useState, useCallback, useMemo } from 'react';
import { useROSConnection } from './useROSConnection';
import { STREAM_DEFAULTS } from '@/lib/utils/constants';

/**
 * Hook for managing camera stream URLs from web_video_server.
 *
 * Builds MJPEG stream URLs of the form:
 *   http://{roverIP}:{videoPort}/stream?topic={topic}&type=mjpeg&quality=80&width=640&height=480
 *
 * @param {string} topic - ROS image topic (e.g., '/camera1/image_raw')
 * @param {Object} [options] - Stream quality options
 * @param {number} [options.quality] - JPEG quality 1-100
 * @param {number} [options.width] - Stream width in pixels
 * @param {number} [options.height] - Stream height in pixels
 * @returns {Object} Stream state and controls
 */
export function useCameraStream(topic, options = {}) {
  const { roverIP, videoPort, isConnected } = useROSConnection();
  const [streamError, setStreamError] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  const quality = options.quality ?? STREAM_DEFAULTS.quality;
  const width = options.width ?? STREAM_DEFAULTS.width;
  const height = options.height ?? STREAM_DEFAULTS.height;

  const streamUrl = useMemo(() => {
    if (!topic || !roverIP || !isConnected) return null;

    // Build query string manually - topic should NOT be URL-encoded
    // web_video_server expects forward slashes as-is in topic names
    const queryParams = [
      `topic=${topic}`,
      `type=${STREAM_DEFAULTS.type}`,
      `quality=${quality}`,
      `width=${width}`,
      `height=${height}`,
    ].join('&');

    return `http://${roverIP}:${videoPort}/stream?${queryParams}`;
  }, [topic, roverIP, videoPort, isConnected, quality, width, height]);

  const snapshotUrl = useMemo(() => {
    if (!topic || !roverIP || !isConnected) return null;

    // Build query string manually - topic should NOT be URL-encoded
    const queryParams = [
      `topic=${topic}`,
      `quality=${quality}`,
      `width=${width}`,
      `height=${height}`,
    ].join('&');

    return `http://${roverIP}:${videoPort}/snapshot?${queryParams}`;
  }, [topic, roverIP, videoPort, isConnected, quality, width, height]);

  const handleLoad = useCallback(() => {
    setIsLoading(false);
    setStreamError(false);
  }, []);

  const handleError = useCallback(() => {
    setIsLoading(false);
    setStreamError(true);
  }, []);

  const retry = useCallback(() => {
    setStreamError(false);
    setIsLoading(true);
  }, []);

  return {
    streamUrl,
    snapshotUrl,
    isLoading,
    streamError,
    isAvailable: !!streamUrl,
    handleLoad,
    handleError,
    retry,
  };
}

export default useCameraStream;
