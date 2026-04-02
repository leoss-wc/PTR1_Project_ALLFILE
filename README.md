# ptR1 — Autonomous Indoor Patrol Robot

> Senior Year Thesis Project | Naresuan University | 2024–2026

ptR1 is an autonomous indoor patrol robot designed for long-corridor environments. It combines ROS-based navigation, real-time object detection, and a full-stack remote control application — all running on edge hardware.

---

## Features

- **Autonomous Navigation** — SLAM-based mapping and path planning using gmapping, AMCL, and TEB Local Planner
- **Real-time Object Detection** — Person detection (COCO pretrained) and door state classification (custom-trained, mAP50 = 0.879) via YOLO11 nano + ONNX Runtime
- **4-Wheel Mecanum Drive** — Omnidirectional movement with PID motor control
- **Remote Control App** — Electron-based dashboard with live video streaming (WebRTC/RTSP), map visualization, and navigation commands
- **Remote Access** — Tailscale VPN for out-of-network access

---

## System Architecture

```
┌─────────────────────────────────────────────┐
│              Operator PC                    │
│         Electron App (Node.js)              │
│   Map View │ Video Stream │ Nav Commands    │
└──────────────────┬──────────────────────────┘
                   │ WebSocket (rosbridge)
                   │ WebRTC / RTSP
┌──────────────────▼──────────────────────────┐
│           Raspberry Pi 4B                   │
│              ROS Noetic                     │
│  SLAM │ AMCL │ TEB │ YOLO11 (ONNX Runtime) │
└──────────────────┬──────────────────────────┘
                   │ rosserial (USB)
┌──────────────────▼──────────────────────────┐
│              ESP32-S3                       │
│   Motor Control │ Relay │ IMU │ LED Status  │
└─────────────────────────────────────────────┘
```

---

## Hardware

| Component | Spec |
|---|---|
| Main Computer | Raspberry Pi 4B (4GB RAM) |
| Microcontroller | ESP32-S3 |
| Drive System | 4-Wheel Mecanum |
| LiDAR | YDLidar G2 |
| Camera | Fisheye USB Camera OVA5647 |
| Motor Driver | TB6612FNG via PCF8575 |

---

## Software Stack

| Layer | Technology |
|---|---|
| Robot OS | ROS Noetic (Ubuntu 20.04) |
| SLAM | slam_toolbox (async) |
| Localization | AMCL |
| Local Planner | TEB Local Planner |
| Object Detection | YOLO11 nano + ONNX Runtime |
| Firmware | ESP32-S3 (Arduino / rosserial) |
| Control App | Electron + Node.js + WebRTC |

---

## Installation

### Prerequisites

- ROS Noetic on Ubuntu 20.04
- Python 3.8+
- Node.js 18+
- Electron

### ROS Setup (Raspberry Pi)

```bash
# Clone the repository
git clone https://github.com/leoss-wc/ptR1.git
cd ptR1

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
catkin_make
source devel/setup.bash
```

### Launch Base System (Raspberry Pi)

```bash
roslaunch ptR1_navigation base_sysptR1.launch
```

This starts all core nodes: rosbridge, serial connection, TF publishers, map manager, navigation manager, stream manager, and system monitor.

---

## Usage

### 1. Create a Map (SLAM Mode)

```bash
# Start SLAM via ROS service
rosservice call /map_manager/start_slam

# Drive the robot around to build the map
# When done, save the map
rosservice call /map_manager/save_map "name: 'my_map'"

# Stop SLAM
rosservice call /map_manager/stop_processes
```

### 2. Start Navigation

```bash
# Step 1: Load a map (starts map_server)
rosservice call /map_manager/select_nav_map "name: 'my_map'"

# Step 2: Start AMCL + move_base
rosservice call /nav/start "restore_pose: true"
```

### 3. Send Navigation Goals

**Option A — Direct ROS service (Patrol mode)**
```bash
# Start patrol with waypoints and looping
rosservice call /nav/start_patrol "goals: [...], loop: true"

# Pause / Resume / Stop patrol
rosservice call /nav/pause_patrol
rosservice call /nav/resume_patrol
rosservice call /nav/stop_patrol
```

**Option B — Via Electron App**

Open the control app, place waypoints on the map, and use the patrol controls in the dashboard.

### 4. Home Position

```bash
# Set current position as home
rosservice call /nav/set_home "name: 'my_map'"

# Navigate back to home
rosservice call /nav/go_home "name: 'my_map'"
```

### Electron App (Operator PC)

```bash
cd ptR1-app
npm install
npm start
```



---

## Detection Models

| Model | Task | Dataset | Confidence | mAP50 |
|---|---|---|---|---|
| Model 1 | Person Detection | COCO (pretrained) | 0.45 | — |
| Model 2 | Door State (open/close) | Custom (1,197 images) | 0.30 | 0.879 |

---

## Author

**Thirayut Wanchiang**
Computer Engineering, Naresuan University
[github.com/leoss-wc](https://github.com/leoss-wc) | thirayut.wc@gmail.com