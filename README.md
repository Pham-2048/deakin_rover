<div align="center">

![Deakin Rover Logo](docs/images/logo.png)

# Deakin Rover — Australian Rover Challenge 2026

**A student-built, ROS2-powered lunar rover competing at the Australian Rover Challenge 2026.**

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/Docker-containerised-2496ED?logo=docker&logoColor=white)](https://www.docker.com/)
[![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Docs](https://img.shields.io/badge/docs-ReadTheDocs-8CA1AF?logo=readthedocs)](https://deakin-rover.readthedocs.io)
[![Build](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com/mirmisbahali/deakin-rover/actions)

</div>

---

<!-- ![Rover hero shot](docs/images/rover_hero.jpg)
*The Deakin Rover ready for field deployment at ARC 2026.* -->

---

## Table of Contents

- [Project Overview](#-project-overview)
- [System Architecture](#-system-architecture)
- [Features](#-features)
- [Getting Started](#-getting-started)
- [Configuration](#-configuration)
- [Team & Acknowledgements](#-team--acknowledgements)
- [License](#-license)

---

## 🚀 Project Overview

The Deakin Rover is designed and built by the [Deakin Robotics Club](https://www.deakin.edu.au) for the **[Australian Rover Challenge (ARC)](https://www.australianroverchallenge.com.au/)**, Australia's premier university rover competition. Teams design, build, and operate rovers through a series of field tasks including terrain traversal, equipment servicing, autonomous navigation, and science sample retrieval.

This repository contains the full software stack of Deakin Rover Borealis for the 2026 competition entry: onboard Jetson Nano software, base station ROS2 workspace, and the web-based operator dashboard built with Next.js — all containerised with Docker.

---

## 🏗 System Architecture

<!-- ![System architecture diagram](docs/images/architecture.png)
*High-level architecture: rover onboard stack (left) communicating with the operator base station (right) over Wi-Fi.* -->

The rover runs a self-contained ROS2 stack on an NVIDIA Jetson Nano. The operator connects via a Next.js web GUI over Wi-Fi with no ROS2 installation required on the operator's PC.

> **Note on `dcr_base_station/base_station_ws/` and `dcr_base_station/.devcontainer/`:** These exist solely for local development and GUI testing. They allow a developer without access to the physical rover to spin up an Ubuntu + ROS2 Jazzy container running the same nodes as the Jetson, so the GUI can be validated end-to-end on a laptop. They are **not** part of the competition architecture and are **not** deployed on-site.

**Rover (NVIDIA Jetson Nano — `dcr_rover/`):**

```
USB Joystick ──► /joy topic ──► dcr_joy_to_motor ──► MotorMovementCommand.srv ──► BLD-305s (RS485/Modbus RTU)
                                                                                        └── 6-wheel drivetrain

                              motor_node (IKPy FK/IK) ──► nobleo_socketcan_bridge ──► CAN bus ──► 6-DOF arm motors

3× USB Cameras ──► mjpg_streamer ──► HTTP :8080 / :8090 / :8091

rover_antenna ──► Serial (/dev/ttyACM1) ──► ESP32 ──► antenna deployment + RGB LED array

rosbridge_server :9090 ◄──── WebSocket ◄──── Next.js GUI (operator PC)
foxglove_bridge  :8765 ◄──── WebSocket ◄──── Foxglove Studio (Realtime visualisation of Robotic Arm digital twin)
```

**Operator PC (`dcr_base_station/gui/` only):**

```
Next.js GUI (localhost:3000)
  └── roslib.js WebSocket ──► rover IP:9090 (rosbridge_server on Jetson)
  └── MJPEG HTTP streams   ──► rover IP:8080 / :8090 / :8091 (mjpg_streamer on Jetson)
```

The Jetson Docker container (`dustynv/ros:jazzy-desktop-r36.4.0-cu128-24.04`, CUDA 12.8-optimised) is the only containerised ROS2 environment used in competition.

---

## ✨ Features

### Software

| Feature | Details |
|---------|---------|
| **ROS2 Jazzy middleware** | rosbridge WebSocket server (:9090) + Foxglove bridge (:8765) for telemetry |
| **Full Docker containerisation** | Dev containers for both Jetson and operator PC; reproducible builds with zero host dependency conflicts |
| **Operator dashboard** | Next.js 16 / React 19 / MUI 7 web GUI — camera feeds, arm control, antenna/LED panel, joystick status |
| **Joystick teleoperation** | USB gamepad → `/joy` → `dcr_joy_to_motor` at 3 Hz with 500 ms watchdog timeout |
| **6-DOF arm kinematics** | `motor_node` implements forward & inverse kinematics via [IKPy](https://github.com/Phylliade/ikpy) from a URDF model |
| **Multi-camera streaming** | 3× USB cameras (640×480, 15 fps, MJPEG) via `mjpg_streamer` on ports 8080/8090/8091 |
| **Deployable antenna** | Custom deployment code + ESP32 firmware; RGB LED control via serial at 115200 baud |
| **SocketCAN bridge** | `nobleo_socketcan_bridge` (C++20) at 1 Mbps — exposes CAN bus as `/socketcan_bridge/rx` and `/socketcan_bridge/tx` ROS2 topics |
| **Custom ROS2 interfaces** | `dcr_interfaces` (motor commands, LED commands), `arm_interfaces` (motor move/status messages) |

### Hardware

| Subsystem | Details |
|-----------|---------|
| **6-wheel drivetrain** | BLD-305s brushless motor controllers via RS485/Modbus RTU; up to 3500 RPM; address-based direction control |
| **6-DOF robotic arm** | CAN-bus actuated; position control and speed control modes; laser-equipped end effector |
| **Multi-camera rig** | 3× USB cameras |
| **Deployable antenna** | ESP32-controlled deployment mechanism with addressable RGB LED array |

<!-- ![Robotic arm in action](docs/images/arm_in_action.jpg)
*6-DOF robotic arm performing a manipulation task.*

![Deployable antenna](docs/images/antenna_deployed.jpg)
*Deployable antenna extended with ESP32-controlled LED array active.* -->

---

## 🛠 Getting Started

Setup involves several prerequisite steps — installing the PEAK CAN driver, patching the Linux kernel for multi-camera USB support, connecting and configuring cameras, installing Docker, VS Code, and the Dev Containers extension — before the ROS2 workspace can be built and launched.

Full step-by-step instructions are in the documentation:

**[📖 Read the full setup guide on ReadTheDocs](https://deakin-rover.readthedocs.io/en/latest/getting-started/)**

![Operator GUI screenshot](docs/images/gui_screenshot.png)
*Operator dashboard: live camera feeds, arm control panel, antenna/LED control, and joystick status.*

---

## ⚙ Configuration

| Parameter | Default | Location |
|-----------|---------|---------|
| CAN interface | `can1` | `startRover.sh`, `bringup.launch.xml` |
| CAN bitrate | 1,000,000 bps | `startRover.sh` |
| RS485 port | `/dev/ttyACM0` | `dcr_motor_controller` |
| RS485 baud rate | 9600 | `BLD_305s.py` |
| Max drivetrain speed | 3500 RPM | `dcr_motor_controller` |
| Camera devices | `/dev/video0`, `/dev/video2`, `/dev/video4` | `startCamera.sh` |
| Camera resolution | 640 × 480 @ 15 fps | `cameras.yaml` |
| ESP32 serial port | `/dev/ttyACM1` | `rover_antenna` |
| ESP32 baud rate | 115,200 | `rover_antenna` |
| rosbridge port | 9090 | `bringup.launch.xml` |
| Foxglove bridge port | 8765 | `bringup.launch.xml` |
| Wheel separation | 0.55 m | `controllers.yaml` |
| Wheel radius | 0.20 m | `controllers.yaml` |
| Controller update rate | 5 Hz | `controllers.yaml` |

---

## 👥 Team & Acknowledgements

### Team Deakin — ARC 2026

| Member | GitHub | Title | Contribution |
|--------|--------|-------|--------------|
| Misbah Ali | []() | Software & Autonomous Systems Lead | ROS2 architecture design, Docker containerisation, teleoperation pipeline, GUI development |
| Ayan | []() | Software Team | GUI development |
| Atharva | []() | Software Team | GUI development |
| Bon | []() | Payloads Team Lead | Robotic arm forward & inverse kinematics implementation |
| Nguyen | []() | Cameras & Communication Team Lead | Antenna deployment code and ESP32 firmware for antenna control |
| Tuan | []() | Cameras & Communication Co-lead | Antenna deployment code and ESP32 firmware for antenna control |
| Nathan | []() | Electrical Team Lead | Full electrical architecture (2025 base), drivetrain motor control, Linux kernel patch for multi-camera support, mjpg-streamer integration |

### Acknowledgements

- **Nathan's 2025 Electrical Architecture** — the drivetrain motor control system is built on top of the electrical design and codebase Nathan developed for the 2025 competition. His work on RS485 motor interfacing and the Linux kernel patch for multi-camera USB support laid the foundation for this year's software.
- **[mjpg-streamer](https://github.com/jacksonliam/mjpg-streamer)** — open source MJPEG streaming used for all three camera feeds.
- **[nobleo/nobleo_socketcan_bridge](https://github.com/nobleo/nobleo_socketcan_bridge)** — C++20 SocketCAN ↔ ROS2 bridge used for CAN bus communication with arm motors.

<!-- ![Competition day at ARC 2026](docs/images/arc2026_competition.jpg)
*Team Deakin at the Australian Rover Challenge 2026.* -->

---

## 📚 Resources

| Resource | Link |
|----------|------|
| Full documentation | [deakin-rover.readthedocs.io](https://deakin-rover.readthedocs.io) *(coming soon)* |
| Demo video | [YouTube — Deakin Rover ARC 2026](https://youtube.com) *(coming soon)* |
| Competition results | [ARC 2026 Results](https://www.australianroverchallenge.com.au/) *(coming soon)* |

---

## 📄 License

[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

This project is licensed under the **Apache License 2.0** — see the [LICENSE](LICENSE) file for details.