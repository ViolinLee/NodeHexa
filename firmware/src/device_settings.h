// 通用设备设置（NVS/Preferences 持久化）
// - 仅存放与业务无关、可扩展的配置项
// - 采用缓存 + 显式读写接口，避免 main.cpp 膨胀
#pragma once

#include <Arduino.h>

namespace devsettings {

struct PowerSettings {
  bool lowBatteryProtectionEnabled = true;
};

// 初始化：加载缓存（建议在 setup() 中调用一次）
void init();

// 当前缓存值（无 IO）
bool isLowBatteryProtectionEnabled();
PowerSettings getPowerSettings();

// 写入 NVS 并更新缓存
// 返回 true 表示写入成功
bool setLowBatteryProtectionEnabled(bool enabled);

}  // namespace devsettings

