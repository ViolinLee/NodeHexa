# 🤖 NodeHexa - 智能六足机器人

<div align="center">

![NodeHexa Logo](resource/frontal.jpg)

**一个基于ESP32的智能六足机器人项目，支持Web控制和多种运动模式**

[![Platform](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Framework](https://img.shields.io/badge/Framework-Arduino-green.svg)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Language](https://img.shields.io/badge/Language-C%2B%2B-orange.svg)](https://isocpp.org/)

</div>

## 📖 项目简介

NodeHexa是一个开源的六足机器人项目，基于ESP32微控制器开发。该项目集成了先进的运动控制算法、Web界面控制和实时校准功能，为机器人爱好者和研究人员提供了一个完整的六足机器人解决方案。

## ✨ 核心特性

### 🎮 多种控制方式
- **Web界面控制** - 通过浏览器实时控制机器人运动
- **串口通信** - 支持UART2串口指令控制（调试和外接上位机用）
- **WebSocket通信** - 低延迟的实时数据传输

### 🚀 丰富的运动模式
- **基础移动**: 前进、后退、左转、右转
- **侧向移动**: 左移、右移
- **姿态控制**: X/Y/Z轴旋转、扭转动作
- **特殊动作**: 攀爬、快速前进

### 🔧 精确校准系统
- **实时校准** - 通过Web界面进行舵机角度可视化校准
- **参数保存** - 校准数据自动保存到Flash存储

### 🔋 安全保护功能
- **电池监测** - 实时监控电池电压
- **低电压保护** - 自动LED警告和系统保护

## 🏗️ 技术架构

### 硬件平台
- **主控**: ESP32 (NodeMCU-32S)
- **舵机驱动**: PCA9685 PWM驱动板
- **通信**: WiFi + UART2串口
- **存储**: SPIFFS文件系统

### 软件架构
```
firmware/
├── src/
│   ├── main.cpp          # 主程序入口
│   ├── hexapod.h/cpp     # 六足机器人核心类
│   ├── leg.h/cpp         # 单腿控制
│   ├── movement.h/cpp    # 运动控制算法
│   ├── servo.h/cpp       # 舵机控制
│   └── calibration.h/cpp # 校准系统
├── include/              # 头文件
└── lib/                  # 第三方库
```

## 🎯 运动控制算法

### 运动学计算
- **正运动学**: 根据关节角度计算足端位置
- **逆运动学**: 根据目标位置计算关节角度
- **坐标变换**: 世界坐标系与局部坐标系转换

## 🖼️ 项目展示

<div align="center">

### 结构与硬件设计
![结构设计](resource/45deg.jpg)
*机器人结构*

![PCB电路板](resource/pcb-board.jpg)
*PCB控制板*

![小智拓展板](resource/xiaozhi.jpg)
*小智AI拓展板集成*

</div>

## 🛒 购买链接
- **套件购买**: [NodeHexa 六足机器人套件](https://item.taobao.com/item.htm?ft=t&id=810056770425)

## 🚀 快速开始

### 开发环境要求
- 推荐：安装 PlatformIO 插件的 VSCode IDE

### 连接和配置
1. 机器人开机后连接WiFi热点 "NodeHexa" (密码: roboticscv666)
2. 访问 `http://192.168.4.1` 进入控制界面
3. 进行舵机校准 (访问 `/calibration` 页面)
4. 开始控制机器人运动

## 📱 Web控制界面

### 主控制页面
- **运动控制**: 前进、后退、转向、侧移等
- **姿态控制**: 三轴旋转和扭转动作
- **校准功能**: 一键进入校准模式

### 校准页面
- **实时调整**: 每个舵机的角度微调
- **可视化反馈**: 实时显示调整效果
- **参数保存**: 自动保存校准数据


## 🤝 贡献指南

欢迎提交Issue和Pull Request来改进项目！

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情

## 🙏 致谢

- 基于 [hexapod-v2-7697](https://github.com/SmallpTsai/hexapod-v2-7697) 项目移植
- 参考 [PiHexa18](https://github.com/ViolinLee/PiHexa18) 项目设计
- 感谢所有开源社区的支持

---

<div align="center">

**⭐ 如果这个项目对你有帮助，请给它一个星标！**

Made with ❤️ by ViolinLee

</div>
