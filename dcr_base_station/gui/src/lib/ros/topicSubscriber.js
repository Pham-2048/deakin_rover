import * as ROSLIB from 'roslib';

/**
 * Subscribe to a ROS topic and call a callback on each message.
 * Returns an unsubscribe function.
 *
 * @param {ROSLIB.Ros} ros - Active ROS connection
 * @param {string} topicName - Topic name (e.g., '/system/can_status')
 * @param {string} messageType - Message type (e.g., 'std_msgs/Bool')
 * @param {Function} callback - Called with each message
 * @returns {{ topic: ROSLIB.Topic, unsubscribe: Function }}
 */
export function subscribeTopic(ros, topicName, messageType, callback) {
  const topic = new ROSLIB.Topic({
    ros,
    name: topicName,
    messageType,
  });

  topic.subscribe(callback);

  return {
    topic,
    unsubscribe: () => topic.unsubscribe(),
  };
}

/**
 * Publish a single message to a ROS topic.
 *
 * @param {ROSLIB.Ros} ros - Active ROS connection
 * @param {string} topicName - Topic name
 * @param {string} messageType - Message type
 * @param {Object} data - Message payload
 */
export function publishMessage(ros, topicName, messageType, data) {
  const topic = new ROSLIB.Topic({
    ros,
    name: topicName,
    messageType,
  });

  topic.publish(data);
}
