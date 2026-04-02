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
│  SLAM │ AMCL │ TEB │ YOLO11 (ONNX Runtime)  │
└──────────────────┬──────────────────────────┘
                   │ rosserial (USB)
┌──────────────────▼──────────────────────────┐
│              ESP32-S3                       │
│   Motor Control │ Relay │ IMU               │
└─────────────────────────────────────────────┘
```

---

## Hardware

| Component | Spec |
|---|---|
| Main Computer | Raspberry Pi 4B (4GB RAM) |
| Microcontroller | ESP32-S3 |
| Drive System | 4-Wheel Mecanum |
| LiDAR | YDG2 LiDar |
| Camera | Fisheye USB Camera OV5647|
| Motor Driver | TB6612FNG via PCF8575 |

---

## Software Stack

| Layer | Technology |
|---|---|
| Robot OS | ROS Noetic (Ubuntu 20.04) |
| SLAM | slam_toolbox |
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
git clone https://github.com/leoss-wc/ptR1_bot.git
cd ptR1

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
catkin_make
source devel/setup.bash
```

### Launch Navigation

```bash
# Start the base system 
roslaunch ptR1 navigation_base.launch
# Start the full navigation stack
rosrun ptR1 navigation_node.py
#send service  
# Start object detection node
rosrun stream_manager_node.py
#send service  
```

### Electron App (Operator PC)

```bash
cd ptR1-app
npm install
npm start
```

---

## Usage

1. Power on the robot and connect to the same network (or via Tailscale)
2. Launch ROS navigation stack on Raspberry Pi
3. Open the Electron control app on your PC
4. Use the map panel to set navigation goals
5. Monitor live video stream and detection results in real-time

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
