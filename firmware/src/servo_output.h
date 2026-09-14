#pragma once

#include <stdint.h>

namespace hexapod { namespace hal { class PCA9685; } }

namespace servo_output {

struct OutputState {
  bool writeAllowed;
  bool released;
  uint32_t writeFailures;
  uint32_t releaseFailures;
  uint8_t controllerCount;
};

bool registerController(hexapod::hal::PCA9685* controller);
bool write(hexapod::hal::PCA9685& controller, uint8_t channel, uint16_t on, uint16_t off);
bool releaseAll();
bool writeAllowed();
OutputState state();

}  // namespace servo_output
