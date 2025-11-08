// AP 配置与状态机（NVS/Preferences 持久化）
#pragma once

#include <Arduino.h>

namespace apconfig {

struct APConfig {
  String ssid;
  String password;
  bool pending;
  String prevSsid;
  String prevPassword;
};

// 初始化：加载配置、处理 pending 监控、启动 AP
void init();

// 当前 SSID（运行中）
String getCurrentSSID();

// 是否处于待确认状态
bool isPending();

// 当用户已在新 AP 下访问到页面时调用，清除 pending
void confirm();

// 设置新配置（写入 NVS，标记 pending，保存 prev_*）
// 返回 true 表示已接受（将在稍后重启生效）
bool setNewConfig(const String& ssid, const String& password);

// 恢复为默认配置（清除 pending 与 prev_*），并重启生效
void resetToDefault();

// 安排一次延迟重启
void requestReboot(uint32_t delayMs = 1000);

// 串口打印当前 AP 信息
void printCurrentAPInfo(Stream& out);

// 获取完整配置（用于 API 返回）
APConfig getConfig();

// 在页面访问处调用，用于自动确认（若处于 pending）
inline void autoConfirmIfPending() {
  if (isPending()) confirm();
}

}  // namespace apconfig


