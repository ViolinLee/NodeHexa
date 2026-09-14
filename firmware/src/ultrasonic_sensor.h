#pragma once

#include <stdint.h>

namespace ultrasonic {

struct Reading {
  bool valid;
  bool pending;
  uint32_t distanceMm;
  uint32_t pulseWidthUs;
  uint32_t sampleCount;
  uint32_t timeoutCount;
  uint32_t errorCount;
  uint32_t measuredAtMs;
  uint32_t ageMs;
};

bool begin();
bool requestMeasurement(uint32_t timeoutMs = 40);
bool measureOnce(Reading& out, uint32_t timeoutMs = 40);
Reading latest();
bool online(uint32_t maxAgeMs = 5000);

}  // namespace ultrasonic
