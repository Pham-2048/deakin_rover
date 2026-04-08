#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <modbus/modbus.h>
#include <map>
#include <string>
#include <vector>

namespace rover_motor_driver
{

struct MotorConfig {
  int address;          // Modbus slave address (1–6)
  bool invert;          // Flip direction for this wheel
};

class BLD305SHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  modbus_t * ctx_{nullptr};

  std::string serial_port_;
  int baud_rate_{9600};
  double max_rpm_{7000.0};

  std::vector<MotorConfig> motors_;           // one per joint, in joint order
  std::vector<double> hw_commands_;           // velocity commands (rad/s)
  std::vector<double> hw_velocities_;         // state: actual velocity (rad/s)
  std::vector<double> hw_positions_;          // state: integrated position (rad)
  std::map<int, uint16_t> prev_ctrl_map_;     // cached direction per unique address
  std::map<int, uint16_t> prev_speed_map_;    // cached speed per unique address
};

}  // namespace rover_motor_driver
