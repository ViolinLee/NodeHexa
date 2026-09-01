#include "power_policy.h"

#include <string.h>

namespace power {

bool isValidPolicyValue(uint8_t raw) {
  return raw <= static_cast<uint8_t>(LowBatteryPolicy::WarnOnly);
}

LowBatteryPolicy policyFromStoredValue(uint8_t raw, LowBatteryPolicy fallback) {
  return isValidPolicyValue(raw) ? static_cast<LowBatteryPolicy>(raw) : fallback;
}

LowBatteryPolicy policyFromLegacyProtection(bool enabled) {
  return enabled ? LowBatteryPolicy::ReleasePwm : LowBatteryPolicy::WarnOnly;
}

bool policyBlocksMotion(LowBatteryPolicy policy) {
  return policy != LowBatteryPolicy::WarnOnly;
}

bool policyReleasesPwm(LowBatteryPolicy policy) {
  return policy == LowBatteryPolicy::ReleasePwm;
}

const char* policyToString(LowBatteryPolicy policy) {
  switch (policy) {
    case LowBatteryPolicy::ReleasePwm:
      return "release_pwm";
    case LowBatteryPolicy::HoldAndLock:
      return "hold_and_lock";
    case LowBatteryPolicy::WarnOnly:
      return "warn_only";
  }
  return "release_pwm";
}

bool policyFromString(const char* value, LowBatteryPolicy& policy) {
  if (!value) {
    return false;
  }
  if (strcmp(value, "release_pwm") == 0) {
    policy = LowBatteryPolicy::ReleasePwm;
    return true;
  }
  if (strcmp(value, "hold_and_lock") == 0) {
    policy = LowBatteryPolicy::HoldAndLock;
    return true;
  }
  if (strcmp(value, "warn_only") == 0) {
    policy = LowBatteryPolicy::WarnOnly;
    return true;
  }
  return false;
}

}  // namespace power
