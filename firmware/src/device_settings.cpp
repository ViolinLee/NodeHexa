// 通用设备设置（NVS/Preferences 持久化）

#include "device_settings.h"

#include <Preferences.h>

namespace devsettings {

namespace {

Preferences prefs;

// NVS 命名空间与键名
static constexpr const char* kNs = "settings";
static constexpr const char* kKeyLowBatteryProtect = "lb_protect";

// 缓存值（避免频繁 NVS IO）
bool cachedLowBatteryProtectionEnabled = true;

void loadFromNvs() {
  // 只读打开若命名空间尚未创建会返回 false，首次需回退到读写创建
  if (!prefs.begin(kNs, true)) {
    prefs.begin(kNs, false);
  }
  cachedLowBatteryProtectionEnabled = prefs.getBool(kKeyLowBatteryProtect, true);
  prefs.end();
}

bool writeLowBatteryProtectionToNvs(bool enabled) {
  if (!prefs.begin(kNs, false)) {
    return false;
  }
  prefs.putBool(kKeyLowBatteryProtect, enabled);
  prefs.end();
  return true;
}

}  // namespace

void init() {
  loadFromNvs();
}

bool isLowBatteryProtectionEnabled() {
  return cachedLowBatteryProtectionEnabled;
}

PowerSettings getPowerSettings() {
  PowerSettings s;
  s.lowBatteryProtectionEnabled = cachedLowBatteryProtectionEnabled;
  return s;
}

bool setLowBatteryProtectionEnabled(bool enabled) {
  if (!writeLowBatteryProtectionToNvs(enabled)) {
    return false;
  }
  cachedLowBatteryProtectionEnabled = enabled;
  Serial.printf("Settings: lowBatteryProtectionEnabled=%s\n", enabled ? "true" : "false");
  return true;
}

}  // namespace devsettings

