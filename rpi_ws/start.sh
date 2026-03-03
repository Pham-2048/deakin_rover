#!/bin/bash
# ============================================================
# Deakin Rover — One-Click Startup Script
# ============================================================
#
# Just run this script and the entire rover system starts up.
# No Linux knowledge required!
#
# USAGE:
#   ./start.sh              # Auto-detects mock vs real hardware
#   ./start.sh --mock       # Force mock mode (no hardware)
#   ./start.sh --real       # Force real hardware mode
#   ./start.sh --help       # Show this help
#
# WHAT IT DOES:
#   1. Builds the ROS 2 workspace (first time only)
#   2. Sets up CAN bus if hardware is detected
#   3. Launches all rover nodes
#   4. Prints the IP address for GUI connection
#
# TO STOP: Press Ctrl+C
# ============================================================

set -e

# -- Colors for pretty output --
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m' # No Color

print_banner() {
    echo ""
    echo -e "${CYAN}${BOLD}"
    echo "  ____  _____    _    _  _____ _   _   ____   _____     _______ ____  "
    echo " |  _ \| ____|  / \  | |/ /_ _| \ | | |  _ \ / _ \ \   / / ____|  _ \ "
    echo " | | | |  _|   / _ \ | ' / | ||  \| | | |_) | | | \ \ / /|  _| | |_) |"
    echo " | |_| | |___ / ___ \| . \ | || |\  | |  _ <| |_| |\ V / | |___|  _ < "
    echo " |____/|_____/_/   \_\_|\_\___|_| \_| |_| \_\\___/  \_/  |_____|_| \_\\"
    echo -e "${NC}"
    echo ""
}

info()    { echo -e "${GREEN}[INFO]${NC}  $1"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $1"; }
error()   { echo -e "${RED}[ERROR]${NC} $1"; }
step()    { echo -e "\n${CYAN}${BOLD}>>> $1${NC}"; }

# -- Parse arguments --
MODE=""
for arg in "$@"; do
    case $arg in
        --mock)  MODE="mock"  ;;
        --real)  MODE="real"  ;;
        --help|-h)
            echo "Deakin Rover Startup Script"
            echo ""
            echo "Usage: ./start.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --mock    Force mock mode (simulated hardware)"
            echo "  --real    Force real hardware mode (CAN, cameras, RS485)"
            echo "  --help    Show this help message"
            echo ""
            echo "If no option is given, the script auto-detects hardware."
            exit 0
            ;;
        *)
            error "Unknown option: $arg"
            echo "Run ./start.sh --help for usage."
            exit 1
            ;;
    esac
done

print_banner

# -- Determine workspace directory --
# Works whether you run from rover_ws/ or the repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROVER_WS="$SCRIPT_DIR"

# Verify we're in the right place
if [ ! -d "$ROVER_WS/src" ]; then
    error "Cannot find src/ directory. Run this script from rover_ws/."
    exit 1
fi

# ============================================================
# Step 1: Source ROS 2
# ============================================================
step "Step 1/5: Sourcing ROS 2 environment..."

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    info "ROS 2 Jazzy sourced."
else
    error "ROS 2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    error "Are you running inside the Docker container?"
    echo ""
    echo "  To start the container, open this project in VS Code"
    echo "  and select the 'rpi ros2 jazzy' dev container."
    exit 1
fi

# ============================================================
# Step 2: Build workspace (if needed)
# ============================================================
step "Step 2/5: Building workspace..."

cd "$ROVER_WS"

if [ -f "$ROVER_WS/install/setup.bash" ]; then
    info "Workspace already built. Skipping build."
    info "(To force rebuild, delete the install/ directory and re-run)"
else
    info "First-time build — this may take a few minutes..."
    echo ""

    # rover_interfaces must build first (generates service types)
    info "Building rover_interfaces first (generates service types)..."
    colcon build --packages-select rover_interfaces --symlink-install 2>&1 | tail -5
    source install/setup.bash

    info "Building all remaining packages..."
    colcon build --symlink-install 2>&1 | tail -5

    if [ $? -eq 0 ]; then
        info "Build successful!"
    else
        error "Build failed. Check the output above for errors."
        exit 1
    fi
fi

source "$ROVER_WS/install/setup.bash"
info "Workspace sourced."

# ============================================================
# Step 3: Auto-detect hardware mode
# ============================================================
step "Step 3/5: Detecting hardware..."

USE_MOCK="true"

if [ "$MODE" = "mock" ]; then
    USE_MOCK="true"
    info "Mock mode forced via --mock flag."
elif [ "$MODE" = "real" ]; then
    USE_MOCK="false"
    info "Real hardware mode forced via --real flag."
else
    # Auto-detect: check for CAN interface or USB cameras
    HAS_CAN=false
    HAS_CAMERA=false

    # Check for CAN interface
    if ip link show can0 &>/dev/null; then
        HAS_CAN=true
    fi

    # Check for USB cameras
    if ls /dev/video* &>/dev/null; then
        HAS_CAMERA=true
    fi

    if $HAS_CAN || $HAS_CAMERA; then
        USE_MOCK="false"
        info "Hardware detected! Running in REAL hardware mode."
        $HAS_CAN    && info "  CAN interface: can0 found"
        $HAS_CAMERA && info "  USB cameras: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
    else
        USE_MOCK="true"
        info "No hardware detected. Running in MOCK mode (simulated)."
    fi
fi

echo ""
if [ "$USE_MOCK" = "true" ]; then
    echo -e "  Mode: ${YELLOW}${BOLD}MOCK${NC} (simulated hardware — test patterns, fake sensors)"
else
    echo -e "  Mode: ${GREEN}${BOLD}REAL HARDWARE${NC} (CAN bus, USB cameras, RS485)"
fi

# ============================================================
# Step 4: Set up CAN bus (real hardware only)
# ============================================================
step "Step 4/5: Hardware setup..."

if [ "$USE_MOCK" = "false" ]; then
    # Set up CAN bus if interface exists but isn't up
    if ip link show can0 &>/dev/null; then
        CAN_STATE=$(ip -brief link show can0 | awk '{print $2}')
        if [ "$CAN_STATE" != "UP" ]; then
            info "Bringing up CAN bus (1 Mbps)..."
            sudo ip link set can0 type can bitrate 1000000 2>/dev/null || true
            sudo ip link set can0 up 2>/dev/null || true
            if ip link show can0 | grep -q "UP"; then
                info "CAN bus is UP."
            else
                warn "Could not bring up CAN bus. Arm motors may not work."
            fi
        else
            info "CAN bus already UP."
        fi
    else
        warn "No CAN interface found. Arm motors will not work."
        warn "If you have a CAN adapter, run: sudo modprobe can && sudo modprobe can_raw"
    fi
else
    info "Mock mode — skipping hardware setup."
fi

# ============================================================
# Step 5: Launch the rover system
# ============================================================
step "Step 5/5: Launching rover system..."

# Get the IP address for GUI connection
ROVER_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
if [ -z "$ROVER_IP" ]; then
    ROVER_IP="<unknown>"
fi

echo ""
echo -e "${GREEN}${BOLD}============================================${NC}"
echo -e "${GREEN}${BOLD}  ROVER IS STARTING UP!${NC}"
echo -e "${GREEN}${BOLD}============================================${NC}"
echo ""
echo -e "  ${BOLD}To connect the GUI:${NC}"
echo -e "    1. On your laptop, run:  ${CYAN}cd base_station_ws/gui && npm run dev${NC}"
echo -e "    2. Open:                 ${CYAN}http://localhost:3000${NC}"
echo -e "    3. Enter Rover IP:       ${CYAN}${ROVER_IP}${NC}"
echo -e "    4. Port:                 ${CYAN}9090${NC}"
echo -e "    5. Video Port:           ${CYAN}8080${NC}"
echo ""
echo -e "  ${BOLD}To stop:${NC} Press ${RED}Ctrl+C${NC}"
echo ""
echo -e "${GREEN}${BOLD}============================================${NC}"
echo ""

# Launch everything
ros2 launch rover_bringup rover.launch.py use_mock:="$USE_MOCK"
