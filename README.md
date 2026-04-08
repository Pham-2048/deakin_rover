# Deakin Rover

Deakin Rover is a student-built rover developed by the Deakin Robotics Club for the Australian Rover Challenge.

## Repository Structure

```
deakin_rover/
├── dcr_rover/          # ROS2 software running on the Jetson Nano (rover onboard computer)
└── dcr_base_station/   # ROS2 software and web GUI running on the operator's base station PC
```

## dcr_rover

ROS2 workspace (`rover_ws/`) deployed on the Jetson Nano. Contains packages for:

- Drive motor control via CAN bus
- Robotic arm control
- Camera streaming
- Rover bringup and launch files

## dcr_base_station

ROS2 workspace (`base_station_ws/`) and a Next.js web GUI (`gui/`) deployed on the operator's PC. Contains packages for:

- Joystick-to-motor command translation
- Motor controller interface
- Web-based operator dashboard (camera feeds, status, arm control)

## Setup

See the `README.md` inside each sub-directory for setup and build instructions.

## License

Apache License 2.0 — see [LICENSE](LICENSE).
