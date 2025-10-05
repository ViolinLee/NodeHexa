# 🤖 NodeHexa - 智能六足机器人

<div align="center">

![NodeHexa Logo](resource/frontal.jpg)

**一个基于ESP32的智能六足机器人项目，支持Web控制和多种运动模式**

[![Platform](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Framework](https://img.shields.io/badge/Framework-Arduino-green.svg)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-GPL--3.0-red.svg)](LICENSE)
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
<div style="display: flex; justify-content: space-between; margin-bottom: 10px;">
  <div style="text-align: center; width: 33%;">
    <img src="resource/45deg.jpg" alt="结构设计" style="width: 100%;">
    <p><em>机器人结构</em></p>
  </div>
  <div style="text-align: center; width: 33%;">
    <img src="resource/pcb-board.jpg" alt="PCB电路板" style="width: 100%;">
    <p><em>PCB控制板</em></p>
  </div>
  <div style="text-align: center; width: 33%;">
    <img src="resource/xiaozhi.jpg" alt="小智拓展板" style="width: 100%;">
    <p><em>小智AI拓展板</em></p>
  </div>
</div>

</div>

## 🛒 购买链接
- **套件购买**: [NodeHexa 六足机器人套件](https://item.taobao.com/item.htm?ft=t&id=810056770425)

## 📱 复刻教程

<div align="center">

![公众号二维码](resource/qrcode_8cm.jpg)   
*扫描上方二维码，关注**公众号**获取六足机器人复刻教程及更多技术资源*

</div>

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


## 🎯 作者

- **B站**: [@智造师_RoboticsCV](https://space.bilibili.com/智造师_RoboticsCV)
- **GitHub**: [@ViolinLee](https://github.com/ViolinLee)
- **微信公众号**: RoboticsCV

## 🤝 贡献指南

欢迎提交Issue和Pull Request来改进项目！

如果您使用本项目进行二次开发或商业应用，请遵守GPL-3.0协议的要求。

## 📄 开源协议

本项目采用 **GNU General Public License v3.0 (GPL-3.0)** 开源协议。

### 主要条款：
- ✅ **允许**：商业使用、修改、分发、专利使用、私人使用
- ⚠️ **要求**：
  - 披露源代码：基于本项目的修改版本必须开源
  - 协议和版权声明：必须保留原始协议和版权信息
  - 相同协议分发：衍生作品必须使用相同的GPL-3.0协议
  - 说明修改：需要说明对原代码的修改内容
- ❌ **禁止**：不提供任何担保和责任

详细内容请查看 [LICENSE](LICENSE) 文件或访问 [GNU GPL-3.0官网](https://www.gnu.org/licenses/gpl-3.0.html)

## 🙏 致谢

- 基于 [hexapod-v2-7697](https://github.com/SmallpTsai/hexapod-v2-7697) 项目移植
- 参考 [PiHexa18](https://github.com/ViolinLee/PiHexa18) 项目设计
- 感谢所有开源社区的支持

---

<div align="center">

**⭐ 如果这个项目对你有帮助，请给它一个星标！**

**📺 关注B站 [@智造师_RoboticsCV](https://space.bilibili.com/智造师_RoboticsCV) 获取更多机器人项目**

**💬 关注公众号 `RoboticsCV` 获取技术文章和教程**

Made with ❤️ by [ViolinLee](https://github.com/ViolinLee)

---

Copyright © 2024 ViolinLee. Licensed under GPL-3.0.

</div>
