// 通用设备设置（NVS/Preferences 持久化）
// - 仅存放与业务无关、可扩展的配置项
// - 采用缓存 + 显式读写接口，避免 main.cpp 膨胀
#pragma once

#include <Arduino.h>

namespace devsettings {

enum class MotionButtonMode : uint8_t {
  Continuous = 0,
  SingleCycle = 1,
};

struct PowerSettings {
  bool lowBatteryProtectionEnabled = true;
};

struct MotionSettings {
  MotionButtonMode buttonMode = MotionButtonMode::Continuous;
};

// 初始化：加载缓存（建议在 setup() 中调用一次）
void init();

// 当前缓存值（无 IO）
bool isLowBatteryProtectionEnabled();
PowerSettings getPowerSettings();
MotionButtonMode getMotionButtonMode();
MotionSettings getMotionSettings();

// 写入 NVS 并更新缓存
// 返回 true 表示写入成功
bool setLowBatteryProtectionEnabled(bool enabled);
bool setMotionButtonMode(MotionButtonMode mode);

}  // namespace devsettings

