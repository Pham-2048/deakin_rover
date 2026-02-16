// ROS Topic Names
export const TOPICS = {
  CAN_STATUS: '/system/can_status',
  RS485_STATUS: '/system/rs485_status',
  POWER: '/system/power',
  JOYSTICK: '/joy',
  NETWORK: '/system/network',
  NODES: '/system/nodes',
};

// ROS Message Types
export const MSG_TYPES = {
  BOOL: 'std_msgs/msg/Bool',
  STRING: 'std_msgs/msg/String',
  BATTERY_STATE: 'sensor_msgs/msg/BatteryState',
  JOY: 'sensor_msgs/msg/Joy',
  // Custom message types (to be defined in rover packages)
  NETWORK_STATUS: 'std_msgs/msg/String', // JSON-encoded for flexibility
  NODE_LIST: 'std_msgs/msg/String',      // JSON-encoded list of nodes
};

// ROS Service Names
export const SERVICES = {
  LAUNCH_NODE: '/node_manager/launch',
  STOP_NODE: '/node_manager/stop',
  LIST_NODES: '/node_manager/list',
};

// ROS Service Types
export const SRV_TYPES = {
  TRIGGER: 'std_srvs/srv/Trigger',
  SET_BOOL: 'std_srvs/srv/SetBool',
  // Custom service for node management (JSON request/response)
  // Uses rover_interfaces/srv/NodeControl: string data → bool success, string message
  NODE_CONTROL: 'rover_interfaces/srv/NodeControl',
};

// Camera Configuration
export const DEFAULT_VIDEO_PORT = '8080';

export const DEFAULT_CAMERAS = [
  { id: 'cam1', label: 'Front Camera', topic: '/camera1/image_raw' },
  { id: 'cam2', label: 'Rear Camera', topic: '/camera2/image_raw' },
  { id: 'cam3', label: 'Arm Camera', topic: '/camera3/image_raw' },
];

export const STREAM_DEFAULTS = {
  type: 'mjpeg',
  quality: 80,
  width: 640,
  height: 480,
};

// Default configurable nodes that can be launched from the GUI
export const DEFAULT_NODES = [
  { id: 'camera_node', name: 'Camera Node', package: 'rover_camera', executable: 'camera_node' },
  { id: 'motor_ctrl', name: 'Motor Controller', package: 'rover_drive', executable: 'drive_node' },
  { id: 'can_bridge', name: 'CAN Bridge', package: 'rover_can', executable: 'can_bridge_node' },
  { id: 'rs485_bridge', name: 'RS485 Bridge', package: 'rover_serial', executable: 'rs485_bridge_node' },
  { id: 'sensor_hub', name: 'Sensor Hub', package: 'rover_monitor', executable: 'system_monitor_node' },
  { id: 'system_monitor', name: 'System Monitor', package: 'rover_monitor', executable: 'system_monitor_node' },
];
