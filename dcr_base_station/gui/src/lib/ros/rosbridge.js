import * as ROSLIB from 'roslib';

/**
 * ROS Bridge Connection Manager
 * Handles WebSocket connection to rosbridge_server on the rover
 */
class ROSBridge {
  constructor() {
    this.ros = null;
    this.url = null;
    this.isConnected = false;
    this.isConnecting = false;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 10;
    this.reconnectDelay = 1000; // Start with 1 second
    this.maxReconnectDelay = 30000; // Max 30 seconds
    this.reconnectTimeout = null;

    // Event handlers
    this.onConnectionChange = null;
    this.onError = null;
  }

  /**
   * Connect to rosbridge server
   * @param {string} url - WebSocket URL (e.g., 'ws://rover.local:9090')
   * @returns {Promise<ROSLIB.Ros>}
   */
  connect(url) {
    return new Promise((resolve, reject) => {
      if (this.isConnecting) {
        reject(new Error('Connection already in progress'));
        return;
      }

      if (this.isConnected && this.url === url) {
        resolve(this.ros);
        return;
      }

      // Disconnect existing connection if URL changed
      if (this.ros && this.url !== url) {
        this.disconnect();
      }

      this.url = url;
      this.isConnecting = true;

      try {
        this.ros = new ROSLIB.Ros({
          url: url,
        });

        // Connection opened
        this.ros.on('connection', () => {
          console.log('[ROSBridge] Connected to', url);
          this.isConnected = true;
          this.isConnecting = false;
          this.reconnectAttempts = 0;
          this.reconnectDelay = 1000;

          if (this.onConnectionChange) {
            this.onConnectionChange(true);
          }

          resolve(this.ros);
        });

        // Connection error
        this.ros.on('error', (error) => {
          console.error('[ROSBridge] Connection error:', error);
          this.isConnected = false;
          this.isConnecting = false;

          if (this.onError) {
            this.onError(error);
          }

          // Attempt reconnection
          this.scheduleReconnect();

          reject(error);
        });

        // Connection closed
        this.ros.on('close', () => {
          console.log('[ROSBridge] Connection closed');
          this.isConnected = false;
          this.isConnecting = false;

          if (this.onConnectionChange) {
            this.onConnectionChange(false);
          }

          // Attempt reconnection
          this.scheduleReconnect();
        });

      } catch (error) {
        console.error('[ROSBridge] Failed to create connection:', error);
        this.isConnecting = false;
        reject(error);
      }
    });
  }

  /**
   * Disconnect from rosbridge server
   */
  disconnect() {
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout);
      this.reconnectTimeout = null;
    }

    if (this.ros) {
      try {
        this.ros.close();
      } catch (error) {
        console.error('[ROSBridge] Error closing connection:', error);
      }
      this.ros = null;
    }

    this.isConnected = false;
    this.isConnecting = false;
    this.reconnectAttempts = 0;
    this.url = null;
  }

  /**
   * Schedule reconnection with exponential backoff
   */
  scheduleReconnect() {
    // Don't reconnect if we've exceeded max attempts
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.log('[ROSBridge] Max reconnection attempts reached');
      return;
    }

    // Don't reconnect if no URL set or already reconnecting
    if (!this.url || this.isConnecting) {
      return;
    }

    // Clear any existing timeout
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout);
    }

    // Calculate delay with exponential backoff
    const delay = Math.min(
      this.reconnectDelay * Math.pow(2, this.reconnectAttempts),
      this.maxReconnectDelay
    );

    console.log(`[ROSBridge] Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts + 1}/${this.maxReconnectAttempts})`);

    this.reconnectTimeout = setTimeout(() => {
      this.reconnectAttempts++;
      this.connect(this.url).catch((error) => {
        console.error('[ROSBridge] Reconnection failed:', error);
      });
    }, delay);
  }

  /**
   * Get current connection instance
   * @returns {ROSLIB.Ros|null}
   */
  getConnection() {
    return this.ros;
  }

  /**
   * Check if connected
   * @returns {boolean}
   */
  getIsConnected() {
    return this.isConnected;
  }

  /**
   * Get current URL
   * @returns {string|null}
   */
  getUrl() {
    return this.url;
  }

  /**
   * Reset reconnection attempts (useful after manual disconnect/reconnect)
   */
  resetReconnectAttempts() {
    this.reconnectAttempts = 0;
    this.reconnectDelay = 1000;
  }
}

// Singleton instance
const rosBridge = new ROSBridge();

export default rosBridge;
