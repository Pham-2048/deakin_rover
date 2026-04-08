#include "rover_motor_driver/bld305s_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rover_motor_driver
{

// BLD-305S Modbus register addresses
static constexpr uint16_t REG_SPEED        = 0x0056;  // write: direct RPM (0–max_rpm)
static constexpr uint16_t REG_CONTROL      = 0x0066;  // write: 0=stop,1=fwd,2=rev,3=brake
static constexpr uint16_t REG_ACTUAL_SPEED = 0x005F;  // read: actual speed

hardware_interface::CallbackReturn BLD305SHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read global hardware parameters
  serial_port_ = info_.hardware_parameters.at("serial_port");
  baud_rate_   = std::stoi(info_.hardware_parameters.at("baud_rate"));
  max_rpm_     = std::stod(info_.hardware_parameters.at("max_rpm"));

  // Read per-joint parameters
  for (const auto & joint : info_.joints) {
    MotorConfig m;
    m.address = std::stoi(joint.parameters.at("modbus_address"));
    m.invert  = joint.parameters.count("invert") &&
                joint.parameters.at("invert") == "true";
    motors_.push_back(m);
  }

  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);

  // Initialize per-address caches with sentinel values (force first write)
  for (const auto & m : motors_) {
    prev_ctrl_map_[m.address]  = 0xFF;
    prev_speed_map_[m.address] = 0xFFFF;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BLD305SHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  ctx_ = modbus_new_rtu(serial_port_.c_str(), baud_rate_, 'N', 8, 1);
  if (!ctx_) {
    RCLCPP_ERROR(rclcpp::get_logger("BLD305S"),
      "Failed to create Modbus context for %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  modbus_set_response_timeout(ctx_, 0, 30000);   // 30 ms

  if (modbus_connect(ctx_) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("BLD305S"),
      "Cannot connect to RS485 port %s: %s",
      serial_port_.c_str(), modbus_strerror(errno));
    modbus_free(ctx_);
    ctx_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("BLD305S"),
    "Connected to RS485 on %s at %d baud", serial_port_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BLD305SHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Write init registers to each unique address once
  for (auto & [addr, val] : prev_ctrl_map_) {
    modbus_set_slave(ctx_, addr);
    modbus_write_register(ctx_, 0x0136, 0x0001);  // internal control mode
    modbus_write_register(ctx_, 0x0116, 0x0002);  // pole pairs = 2
    modbus_write_register(ctx_, REG_CONTROL, 0);  // stop
    // Reset caches so first write() sends direction and speed to every address
    val = 0xFF;
    prev_speed_map_[addr] = 0xFFFF;
  }
  RCLCPP_INFO(rclcpp::get_logger("BLD305S"), "Motors initialized and stopped.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BLD305SHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & m : motors_) {
    modbus_set_slave(ctx_, m.address);
    modbus_write_register(ctx_, REG_CONTROL, 0);  // stop
  }
  RCLCPP_INFO(rclcpp::get_logger("BLD305S"), "All motors stopped on deactivate.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BLD305SHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // open_loop: true — use commanded velocity as state feedback, no Modbus reads
  for (size_t i = 0; i < motors_.size(); ++i) {
    hw_velocities_[i] = hw_commands_[i];
    hw_positions_[i] += hw_velocities_[i] * period.seconds();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BLD305SHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Deduplicate: build address → (ctrl, speed) using the first joint per address
  std::map<int, std::pair<uint16_t, uint16_t>> addr_cmds;
  for (size_t i = 0; i < motors_.size(); ++i) {
    int addr = motors_[i].address;
    if (addr_cmds.count(addr)) { continue; }  // already set by earlier joint

    double vel = hw_commands_[i];
    if (motors_[i].invert) { vel = -vel; }

    double rpm = std::abs(vel) * 60.0 / (2.0 * M_PI);
    auto speed_val = static_cast<uint16_t>(std::clamp(rpm, 0.0, max_rpm_));

    uint16_t ctrl_val;
    if (vel > 0.05)       ctrl_val = 1;  // forward
    else if (vel < -0.05) ctrl_val = 2;  // reverse
    else                  ctrl_val = 0;  // stop

    addr_cmds[addr] = {ctrl_val, speed_val};
  }

  // Write each unique address once; skip unchanged values
  for (const auto & [addr, cmd] : addr_cmds) {
    auto [ctrl_val, speed_val] = cmd;
    modbus_set_slave(ctx_, addr);
    if (ctrl_val != prev_ctrl_map_[addr]) {
      modbus_write_register(ctx_, REG_CONTROL, ctrl_val);
      prev_ctrl_map_[addr] = ctrl_val;
    }
    if (speed_val != prev_speed_map_[addr]) {
      modbus_write_register(ctx_, REG_SPEED, speed_val);
      prev_speed_map_[addr] = speed_val;
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace rover_motor_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rover_motor_driver::BLD305SHardware,
  hardware_interface::SystemInterface)
