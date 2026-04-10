# 🤖 NodeHexa - Hexapod Robot (with Quadruped Support)

[中文说明](README.md)

<div align="center">

![NodeHexa Logo](resource/frontal.jpg)

**An ESP32-based hexapod robot project with additional quadruped firmware support, featuring Web control, calibration, performance motions, and motion sequence planning.**

[![Platform](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Framework](https://img.shields.io/badge/Framework-Arduino-green.svg)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-GPL--3.0-red.svg)](LICENSE)
[![Language](https://img.shields.io/badge/Language-C%2B%2B-orange.svg)](https://isocpp.org/)

</div>

## 📖 Overview

NodeHexa is an open-source hexapod robot project built on the ESP32 microcontroller. It integrates kinematics algorithms, Web UI control, real-time calibration, and motion sequencing (motion planning), designed for robotics enthusiasts and research purposes.

## 🧩 Supported Configurations

This repository also provides **quadruped firmware support**, allowing both robot types to share one Web UI, calibration flow, and motion-control framework.

| Configuration | Positioning | Scope | Docs / Media Status |
| --- | --- | --- | --- |
| **Hexapod (NodeHexa)** | Primary line | Structure / hardware / firmware / UI / tutorials | Most illustrations in this README are hexapod; full software and hardware design is open-source |
| **Quadruped (NodeQuadMini)** | Additional support | Structure / schematic / firmware / UI | This README includes quadruped images; schematic is available (PCB files are not open-source); tutorials will be updated |

> For structural parts and fastener differences, see `mechanism/README.md` (listed separately for hexapod and quadruped).

## ✨ Key Features

### 🧩 Dual-configuration support (hexapod primary, quadruped additional)
- **Unified abstraction interface** - Hexapod and quadruped share the same control and Web framework
- **Isolated implementation differences** - Hexapod and quadruped are implemented separately without blocking core feature evolution

### 🎮 Multiple control methods
- **Web UI control** - Real-time robot control from a browser
- **Serial communication** - UART2 command control (for debugging and external host integration)
- **WebSocket communication** - Low-latency real-time data transfer
- **Optional voice extension** - Supports the XiaoZhi AI extension board for voice interaction control

### 🚀 Rich motion modes
- **Basic movement**: Forward, backward, turn left, turn right
- **Lateral movement**: Move left, move right
- **Posture control**: X/Y/Z-axis rotation and twisting actions
- **Special actions**: Climbing and fast-forward mode
- **Performance modes**: Freestyle, Beat Sway, and Showtime
- **Motion button mode switching**: Supports both `continuous` and `single-cycle` triggering for basic motions and performance motions
- **Single-leg demo mode (hexapod only)**: Select one leg and demonstrate forward/lateral/lift movement independently
- **Quadruped multi-gait switching**: Trot / Walk / Gallop / Creep
- **Motion sequence planning**: Chain multiple actions with constraints such as cycle/steps/distance/angle, then run in one click (see `/planner`)

### 🔧 Precision calibration system
- **Real-time calibration** - Visual servo-angle calibration from the Web UI
- **Parameter persistence** - Calibration data is automatically saved to Flash storage

### 🔋 Safety protection
- **Battery badge** - Real-time voltage and estimated battery percentage on both the controller and planner pages
- **Battery monitoring** - Real-time battery voltage monitoring
- **Low-voltage protection** - Automatic LED warning and system protection

## 🏗️ Technical Architecture

### Hardware platform
- **MCU**: ESP32 (NodeMCU-32S)
- **Servo driver**: PCA9685 PWM driver board
- **Communication**: WiFi + UART2 serial
- **Storage**: SPIFFS file system
- **Optional voice extension**: XiaoZhi AI extension board

### Software structure
```
firmware/
├── src/
│   ├── main.cpp          # Main entry
│   ├── robot.h           # Unified robot abstraction interface (shared by hexapod/quadruped)
│   ├── hexapod.h/cpp     # Hexapod implementation
│   ├── quad_robot.h/cpp  # Quadruped implementation
│   ├── leg.h/cpp         # Single-leg control (separate/reused for both)
│   ├── movement*.h/cpp   # Motion control algorithms and config tables
│   ├── motion_controller*# Motion sequence / planner execution
│   └── calibration.h/cpp # Calibration system
├── include/              # Header files
└── lib/                  # Third-party libraries
```

## 🎯 Motion Control Algorithms

### Kinematics
- **Forward kinematics**: Calculate foot-end position from joint angles
- **Inverse kinematics**: Calculate joint angles from target position
- **Coordinate transforms**: Conversion between world and local coordinate systems

## 🖼️ Project Showcase

### Hexapod (NodeHexa) structure and hardware design

<table>
  <tr>
    <td align="center">
      <img src="resource/45deg.jpg" alt="Structure design" width="300"/>
      <br/>
      <em>Hexapod Structure</em>
    </td>
    <td align="center">
      <img src="resource/pcb-board.jpg" alt="PCB board" width="300"/>
      <br/>
      <em>PCB Kit</em>
    </td>
    <td align="center">
      <img src="resource/xiaozhi.jpg" alt="XiaoZhi extension board" width="300"/>
      <br/>
      <em>XiaoZhi Extension</em>
    </td>
  </tr>
</table>

### Quadruped (NodeQuadMini) structure and hardware design

<table>
  <tr>
    <td align="center">
      <img src="resource/quad-45deg.png" alt="Quadruped structure" width="300"/>
      <br/>
      <em>Quadruped Structure</em>
    </td>
    <td align="center">
      <img src="resource/quad-frontal.png" alt="Quadruped front view" width="300"/>
      <br/>
      <em>Quadruped Front View</em>
    </td>
    <td align="center">
      <img src="resource/quad-pcb-board.png" alt="Quadruped mainboard" width="300"/>
      <br/>
      <em>Quadruped Mainboard</em>
    </td>
  </tr>
</table>

> Note: The quadruped version currently open-sources the **schematic** for study and maintenance. PCB-related files are not open-source.
>
> The quadruped supports multiple gait modes: Trot / Walk / Gallop / Creep.

## 🛒 Purchase Links
- **Kit purchase**: [NodeHexa Hexapod Kit](https://item.taobao.com/item.htm?ft=t&id=810056770425), [NodeQuadMini Quadruped Kit](https://item.taobao.com/item.htm?id=1022920495655)
- **Controller board purchase**: [NodeHexa Hexapod Controller Board](https://item.taobao.com/item.htm?id=990145258187)
- **Servo purchase (highly recommended)**: [MG90s Servo](https://item.taobao.com/item.htm?id=978672014892)
- **Voice extension board**: [XiaoZhi AI Extension Board (supports hexapod & quadruped)](https://item.taobao.com/item.htm?id=989885356650)

> We strongly recommend the servo listed in our store. There are many MG90s manufacturers on the market, with significant quality and size differences. Low-quality servos are more likely to overheat or burn out, which increases long-term cost. This link is the model selected after our own comparative testing.

<div align="center">
  <img src="resource/step.jpg" alt="Hexapod 3D model" width="400"/>
  <br/>
  <em>Hexapod 3D Model Preview</em>
</div>

<div align="center">
  <img src="resource/quad-step.jpg" alt="Quadruped 3D model" width="400"/>
  <br/>
  <em>Quadruped 3D Model Preview</em>
</div>

## 📱 Build Tutorial

<div align="center">

![WeChat Official Account QR](resource/qrcode_8cm.jpg)  
*Scan the QR code, follow the official account, and send "**六足**" to access detailed build tutorials and the discussion group.*

</div>

Or click **[NodeHexa Tutorials]** below to open the WeChat article list directly:

<details>
<summary><strong>📚 NodeHexa Tutorials</strong></summary>

### Series Tutorials

1. [NodeHexa Build Tutorial (1): Parts Preparation](https://mp.weixin.qq.com/s/QebT1wd3da98jmFbrUHNdA)
2. [NodeHexa Build Tutorial (2): Leg Assembly](https://mp.weixin.qq.com/s/x1spemwsdwfix2QXKvCDqA)
3. [NodeHexa Build Tutorial (3): Body Assembly](https://mp.weixin.qq.com/s/Z3uXM__K4puC-hbytVeSNw)
4. [NodeHexa Build Tutorial (4): Compile & Flash](https://mp.weixin.qq.com/s/InIxQt30JFU6OhD7m3k71Q)
5. [NodeHexa Build Tutorial (5): Functional Debugging](https://mp.weixin.qq.com/s/-viItGeh79Q6JDqvxl3oZQ)

### Feature Demos

- [Open Source Showcase: Hexapod with XiaoZhi AI Voice Interaction + Full Build Tutorial (Bilibili)](https://www.bilibili.com/video/BV19R4gzzEH7)
- [Open Source Showcase: XiaoZhi AI Voice-Controlled Hexapod (WeChat Article)](https://mp.weixin.qq.com/s/sWiMd9wZ3VoEhoh8ss6X7w)

</details>

## 🚀 Quick Start

### Development environment
- Recommended: VSCode IDE with PlatformIO extension installed

### Connection and setup
1. Power on the robot and connect to WiFi hotspot `NodeHexa` (password: `roboticscv666`)
2. Visit `http://192.168.4.1` to open the control panel
3. Perform servo calibration (visit `/calibration`)
4. (Optional) Adjust WiFi AP, low-battery protection, and motion button mode in Settings
5. (Optional) Open motion planner (visit `/planner`) and arrange action sequences
6. Start controlling the robot

## 📱 Web Control Interface

### Main control page
- **Movement control**: Forward, backward, turning, lateral movement, etc.
- **Posture control**: 3-axis rotation and twisting actions
- **Performance modes**: Freestyle, Beat Sway, and Showtime
- **Single-leg demo (hexapod only)**: Select one leg and control lateral/forward/lift movement in real time
- **Battery badge**: Real-time voltage, estimated battery percentage, and low-battery status
- **Settings entry**: Configure WiFi AP, low-battery protection, motion button mode, and view firmware version
- **Calibration entry**: One-click calibration mode
- **Motion planner**: Enter `/planner` to arrange and send motion sequences (single actions can also run directly)

### Calibration page
- **Real-time adjustment**: Fine-tune each servo angle
- **Visual feedback**: See adjustment effects immediately
- **Auto-save**: Calibration data is automatically stored

### Motion planner page (action sequence)
- **Battery display**: Shares the same live voltage and battery-percentage badge as the main page
- **Single action**: Run one constrained action by cycle/steps/distance (posture actions support cycle only)
- **Action sequence**: Run multiple actions in order (up to 5 segments)
- **Queue control**: Supports appending, clearing the queue, and emergency stop

## ❓ FAQ

### 1. Notes on hexapod servo driver boards
**Important**: The left and right servo driver boards use the same circuit design, but there is a small difference during component soldering.

- Only one side should be soldered for the 1x4P and 1x6P pin headers on each servo driver board edge; left and right boards are opposite.
- DIY users must solder the `SJ1` jumper on the left servo driver board (kit users can skip this because it is pre-soldered at factory).

**Why this matters**: After soldering the jumper, the left board gets a different I2C address from the right board, avoiding address conflict. Both boards communicate with the controller over I2C; identical addresses will cause communication issues.

## 🎯 Author

- **Bilibili**: [@智造师_RoboticsCV](https://space.bilibili.com/智造师_RoboticsCV)
- **GitHub**: [@ViolinLee](https://github.com/ViolinLee)
- **WeChat Official Account**: RoboticsCV

## 🤝 Contributing

Issues and Pull Requests are welcome to improve this project.

## 🙏 Acknowledgements

- Secondary development based on [hexapod-v2-7697](https://github.com/SmallpTsai/hexapod-v2-7697)
- Design references from [PiHexa18](https://github.com/ViolinLee/PiHexa18)
- Voice extension based on [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)
- Thanks to all open-source contributors and communities

---

<div align="center">

**⭐ If this project helps you, please give it a star!**

**📺 Follow Bilibili [@智造师_RoboticsCV](https://space.bilibili.com/智造师_RoboticsCV) for more robotics projects**

**💬 Follow WeChat official account `RoboticsCV` for technical articles and tutorials**

Made with ❤️ by [ViolinLee](https://github.com/ViolinLee)

---

Copyright © 2024 ViolinLee. Licensed under GPL-3.0.

</div>
