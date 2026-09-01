// 通用设备设置（NVS/Preferences 持久化）

#include "device_settings.h"

#include <Preferences.h>

namespace devsettings {

namespace {

Preferences prefs;

// NVS 命名空间与键名
static constexpr const char* kNs = "settings";
static constexpr const char* kKeyLowBatteryPolicy = "lb_policy";
static constexpr const char* kKeyLowBatteryProtect = "lb_protect";
static constexpr const char* kKeyMotionButtonMode = "motion_btn";

// 缓存值（避免频繁 NVS IO）
power::LowBatteryPolicy cachedLowBatteryPolicy = power::LowBatteryPolicy::ReleasePwm;
MotionButtonMode cachedMotionButtonMode = MotionButtonMode::Continuous;

void loadFromNvs() {
  // 只读打开若命名空间尚未创建会返回 false，首次需回退到读写创建
  if (!prefs.begin(kNs, true)) {
    prefs.begin(kNs, false);
  }
  const bool hasPolicy = prefs.isKey(kKeyLowBatteryPolicy);
  const bool hasLegacyProtection = prefs.isKey(kKeyLowBatteryProtect);
  if (hasPolicy) {
    const uint8_t rawPolicy = prefs.getUChar(
      kKeyLowBatteryPolicy,
      static_cast<uint8_t>(power::LowBatteryPolicy::ReleasePwm)
    );
    cachedLowBatteryPolicy = power::policyFromStoredValue(
      rawPolicy,
      power::LowBatteryPolicy::ReleasePwm
    );
  } else {
    const bool legacyProtection = hasLegacyProtection
      ? prefs.getBool(kKeyLowBatteryProtect, true)
      : true;
    cachedLowBatteryPolicy = power::policyFromLegacyProtection(legacyProtection);
  }
  uint8_t rawMotionButtonMode = prefs.getUChar(
    kKeyMotionButtonMode,
    static_cast<uint8_t>(MotionButtonMode::Continuous)
  );
  cachedMotionButtonMode = (rawMotionButtonMode == static_cast<uint8_t>(MotionButtonMode::SingleCycle))
    ? MotionButtonMode::SingleCycle
    : MotionButtonMode::Continuous;
  prefs.end();

  if (!hasPolicy) {
    if (prefs.begin(kNs, false)) {
      const size_t written = prefs.putUChar(
        kKeyLowBatteryPolicy,
        static_cast<uint8_t>(cachedLowBatteryPolicy)
      );
      prefs.end();
      if (written != sizeof(uint8_t)) {
        cachedLowBatteryPolicy = power::LowBatteryPolicy::ReleasePwm;
      }
    } else {
      cachedLowBatteryPolicy = power::LowBatteryPolicy::ReleasePwm;
    }
  }
}

bool writeLowBatteryPolicyToNvs(power::LowBatteryPolicy policy) {
  if (!prefs.begin(kNs, false)) {
    return false;
  }
  const size_t written = prefs.putUChar(kKeyLowBatteryPolicy, static_cast<uint8_t>(policy));
  prefs.end();
  return written == sizeof(uint8_t);
}

bool writeMotionButtonModeToNvs(MotionButtonMode mode) {
  if (!prefs.begin(kNs, false)) {
    return false;
  }
  prefs.putUChar(kKeyMotionButtonMode, static_cast<uint8_t>(mode));
  prefs.end();
  return true;
}

}  // namespace

void init() {
  loadFromNvs();
}

power::LowBatteryPolicy getLowBatteryPolicy() {
  return cachedLowBatteryPolicy;
}

PowerSettings getPowerSettings() {
  PowerSettings s;
  s.lowBatteryPolicy = cachedLowBatteryPolicy;
  return s;
}

MotionButtonMode getMotionButtonMode() {
  return cachedMotionButtonMode;
}

MotionSettings getMotionSettings() {
  MotionSettings s;
  s.buttonMode = cachedMotionButtonMode;
  return s;
}

bool setLowBatteryPolicy(power::LowBatteryPolicy policy) {
  if (!power::isValidPolicyValue(static_cast<uint8_t>(policy)) ||
      !writeLowBatteryPolicyToNvs(policy)) {
    return false;
  }
  cachedLowBatteryPolicy = policy;
  Serial.printf("Settings: lowBatteryPolicy=%s\n", power::policyToString(policy));
  return true;
}

bool setMotionButtonMode(MotionButtonMode mode) {
  if (!writeMotionButtonModeToNvs(mode)) {
    return false;
  }
  cachedMotionButtonMode = mode;
  Serial.printf(
    "Settings: motion.buttonMode=%s\n",
    mode == MotionButtonMode::SingleCycle ? "single_cycle" : "continuous"
  );
  return true;
}

}  // namespace devsettings
