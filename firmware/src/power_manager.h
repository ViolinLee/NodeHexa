#pragma once

#include <stdint.h>

#include "power_policy.h"

namespace power_manager {

constexpr uint16_t kWarningThresholdMv = 7200;
constexpr uint16_t kStandbyLatchThresholdMv = 7200;
constexpr uint16_t kMovingLatchThresholdMv = 7000;
constexpr uint8_t kConsecutiveSamplesToLatch = 2;

struct Snapshot {
  uint16_t voltageMv;
  uint8_t consecutiveLowSamples;
  bool lowBatteryLatched;
  bool latchHandled;
  power::LowBatteryPolicy latchedPolicy;
};

struct SampleResult {
  Snapshot state;
  uint16_t thresholdMv;
  bool belowThreshold;
  bool newlyLatched;
};

uint16_t voltageMvFromAdcAverage(uint16_t adcAverage);
SampleResult observeVoltage(uint16_t voltageMv,
                            bool moving,
                            power::LowBatteryPolicy policyAtLatch);
Snapshot snapshot();
bool claimLatchHandling();

}  // namespace power_manager
