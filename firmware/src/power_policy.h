#pragma once

#include <stdint.h>

namespace power {

enum class LowBatteryPolicy : uint8_t {
  ReleasePwm = 0,
  HoldAndLock = 1,
  WarnOnly = 2,
};

bool isValidPolicyValue(uint8_t raw);
LowBatteryPolicy policyFromStoredValue(uint8_t raw, LowBatteryPolicy fallback);
LowBatteryPolicy policyFromLegacyProtection(bool enabled);
bool policyBlocksMotion(LowBatteryPolicy policy);
bool policyReleasesPwm(LowBatteryPolicy policy);
const char* policyToString(LowBatteryPolicy policy);
bool policyFromString(const char* value, LowBatteryPolicy& policy);

}  // namespace power
