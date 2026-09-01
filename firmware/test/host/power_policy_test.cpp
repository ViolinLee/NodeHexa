#include <assert.h>
#include <string.h>

#include "power_policy.h"

int main() {
  using power::LowBatteryPolicy;

  assert(power::policyFromLegacyProtection(true) == LowBatteryPolicy::ReleasePwm);
  assert(power::policyFromLegacyProtection(false) == LowBatteryPolicy::WarnOnly);
  assert(power::policyFromStoredValue(0xff, LowBatteryPolicy::ReleasePwm) ==
         LowBatteryPolicy::ReleasePwm);
  assert(power::policyBlocksMotion(LowBatteryPolicy::ReleasePwm));
  assert(power::policyBlocksMotion(LowBatteryPolicy::HoldAndLock));
  assert(!power::policyBlocksMotion(LowBatteryPolicy::WarnOnly));
  assert(power::policyReleasesPwm(LowBatteryPolicy::ReleasePwm));
  assert(!power::policyReleasesPwm(LowBatteryPolicy::HoldAndLock));
  assert(strcmp(power::policyToString(LowBatteryPolicy::WarnOnly), "warn_only") == 0);

  LowBatteryPolicy parsed = LowBatteryPolicy::ReleasePwm;
  assert(power::policyFromString("hold_and_lock", parsed));
  assert(parsed == LowBatteryPolicy::HoldAndLock);
  assert(!power::policyFromString("invalid", parsed));
  return 0;
}
