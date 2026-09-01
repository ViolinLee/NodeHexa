#include "power_manager.h"

#include <Arduino.h>

namespace power_manager {
namespace {

constexpr float kBatteryVoltageGain = 1.0122f;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
Snapshot state{0, 0, false, false, power::LowBatteryPolicy::ReleasePwm};

}  // namespace

uint16_t voltageMvFromAdcAverage(uint16_t adcAverage) {
  const float voltage = static_cast<float>(adcAverage) * 3.3f / 4095.0f *
                        (100.0f + 47.0f) / 47.0f * kBatteryVoltageGain;
  return static_cast<uint16_t>(voltage * 1000.0f + 0.5f);
}

SampleResult observeVoltage(uint16_t voltageMv,
                            bool moving,
                            power::LowBatteryPolicy policyAtLatch) {
  const uint16_t threshold = moving ? kMovingLatchThresholdMv : kStandbyLatchThresholdMv;
  const bool belowThreshold = voltageMv <= threshold;
  bool newlyLatched = false;

  portENTER_CRITICAL(&stateMux);
  state.voltageMv = voltageMv;
  if (belowThreshold) {
    if (state.consecutiveLowSamples < kConsecutiveSamplesToLatch) {
      ++state.consecutiveLowSamples;
    }
  } else {
    state.consecutiveLowSamples = 0;
  }
  if (state.consecutiveLowSamples >= kConsecutiveSamplesToLatch &&
      !state.lowBatteryLatched) {
    state.lowBatteryLatched = true;
    state.latchHandled = false;
    state.latchedPolicy = policyAtLatch;
    newlyLatched = true;
  }
  const Snapshot copy = state;
  portEXIT_CRITICAL(&stateMux);
  return {copy, threshold, belowThreshold, newlyLatched};
}

Snapshot snapshot() {
  portENTER_CRITICAL(&stateMux);
  const Snapshot copy = state;
  portEXIT_CRITICAL(&stateMux);
  return copy;
}

bool claimLatchHandling() {
  bool claimed = false;
  portENTER_CRITICAL(&stateMux);
  if (state.lowBatteryLatched && !state.latchHandled) {
    state.latchHandled = true;
    claimed = true;
  }
  portEXIT_CRITICAL(&stateMux);
  return claimed;
}

}  // namespace power_manager
