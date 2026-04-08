import * as ROSLIB from 'roslib';

/**
 * Call a ROS service and return the result as a Promise.
 *
 * @param {ROSLIB.Ros} ros - Active ROS connection
 * @param {string} serviceName - Service name (e.g., '/launch_node')
 * @param {string} serviceType - Service type (e.g., 'std_srvs/Trigger')
 * @param {Object} request - Service request payload
 * @returns {Promise<Object>} Service response
 */
export function callService(ros, serviceName, serviceType, request = {}) {
  return new Promise((resolve, reject) => {
    const service = new ROSLIB.Service({
      ros,
      name: serviceName,
      serviceType,
    });

    service.callService(
      request,
      (result) => resolve(result),
      (error) => reject(new Error(error || `Service call to ${serviceName} failed`))
    );
  });
}
