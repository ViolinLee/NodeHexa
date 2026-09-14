# NodeHexa 固件接口参考

本文面向 NodeHexa 与 NodeQuadMini 固件 v3.0.0。硬件代际与固件版本彼此独立；烧录前
应按主板代际选择构建环境。

## 构建环境

| PlatformIO 环境 | 机器人 | 硬件范围 | `/api/caps` |
|---|---|---|---|
| `nodemcu-32s` | NodeHexa | 旧版或未确认版本 | `hardwareRevision=1`、`legacy_or_unknown` |
| `nodequadmini` | NodeQuadMini | 旧版或未确认版本 | `hardwareRevision=1`、`legacy_or_unknown` |
| `nodehexa-v2` | NodeHexaV2 | V2 主板 | `hardwareRevision=2`、`v2` |
| `nodequadmini-v2` | NodeQuadMini V2 | V2 主板 | `hardwareRevision=2`、`v2` |

```bash
cd firmware
platformio run -e nodehexa-v2
platformio run -e nodehexa-v2 --target buildfs
```

应用固件和 SPIFFS 必须使用同一环境构建并一起烧录。发布包内 `version.json` 会记录
`supportedHardware`、`uartProtocolVersion` 和 `legacyUartSupported`。

## 低电量行为

低电量经连续两次采样确认后锁存，只有重新上电才能清除。LED、Web、UART 与蜂鸣器
始终报告锁存事实，设置只决定运动和舵机输出的处置方式。

| 策略 | 运动 | 舵机 PWM | 蜂鸣器 |
|---|---|---|---|
| `release_pwm`（默认） | 阻止 | 全通道关闭，重启前不可恢复 | 持续报警 |
| `hold_and_lock` | 阻止 | 保持最后输出 | 持续报警 |
| `warn_only` | 允许 | 保持输出 | 持续报警 |

开机提示音、未来遥控器连接提示音与低电量报警均由独立任务播放，不阻塞控制循环。低电量
报警优先级最高，不能通过 Web 或串口关闭。

## 外设接线

| 外设 | 主控 GPIO | 说明 |
|---|---:|---|
| UART2 RX / TX | 16 / 17 | 115200 8N1，3.3 V 逻辑 |
| 无源蜂鸣器 | 26 | LEDC 通道 7 |
| 超声波 Trigger | 4 | 输出 |
| 超声波 Echo | 35 | 输入，只允许 3.3 V |

如果超声波模块的 Echo 输出为 5 V，必须增加可靠电平转换或分压，禁止直接连接 GPIO35。
超声波只在 REST 明确请求时执行一次测量，不会自动触发避障或运动。

## UART v2

UART v2 使用二进制外层和 UTF-8 JSON 载荷。多字节整数为小端序，最大载荷 512 字节，
接收半帧超过 250 ms 会丢弃并重新同步。

| 偏移 | 字段 | 长度 |
|---:|---|---:|
| 0 | Magic `A5 4E` | 2 |
| 2 | Version `02` | 1 |
| 3 | Type：HELLO=1、REQUEST=2、RESPONSE=3、EVENT=4、HEARTBEAT=5 | 1 |
| 4 | Flags | 1 |
| 5 | Sequence | 2 |
| 7 | Payload length | 2 |
| 9 | JSON payload | 0–512 |
| 9+N | CRC16-CCITT | 2 |

CRC 参数为 poly `0x1021`、init `0xffff`，覆盖 Version 起至 Payload 末尾。REQUEST 的
RESPONSE 必须使用相同 Sequence；EVENT 不参与请求等待。兼容格式 `$JSON\n` 仍可接收，
但新外设应优先使用 v2。

外设启动后发送 HELLO。`device` 可为 `camera`、`xiaozhi` 或通用设备；主控返回协议、
机器人类型、固件版本和明确的 `power.lowBatteryLatched`。V2 外设超过 6 秒无有效帧会在
能力接口中标记离线。

## 摄像头

图传板通过 UART HELLO 被识别后，主控下发当前 AP 的 SSID 与密码。摄像头仅作为 STA
接入 NodeHexa 网络，不创建用户配置热点。浏览器直接访问摄像头的 81 端口，视频不经过
UART 或主控内存。

- 默认 QVGA，可切换 VGA；JPEG quality 固定为 80。
- 页面进入后台、退出视频模式或摄像头离线时会主动断开 MJPEG。
- 同时只允许一个 MJPEG 客户端。
- 摄像头过流、采集或编码错误通过 `cameraStatus` EVENT 和 `/api/caps` 统计报告。

## REST 能力与诊断

- `GET /api/caps`：机器人、固件、硬件、UART 统计、电源和外设能力。
- `GET /api/settings`、`POST /api/settings`：低电量策略与动作按钮模式。
- `GET /api/ultrasonic`：最近一次测距；增加 `?fresh=1` 异步发起新测量。
- `POST /api/camera/profile`：提交 `{"profile":"qvga"}` 或 `{"profile":"vga"}`。

`/api/ultrasonic?fresh=1` 不等待 Echo，响应中的 `pending` 表示后台测量状态。驱动使用
30 ms 硬超时；未接模块或 Echo 超时会返回 `valid=false` 并增加超时计数。
