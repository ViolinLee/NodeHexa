#include "servo_output.h"

#include <Arduino.h>
#include <atomic>

#include "pwm.h"

namespace servo_output {
namespace {

static constexpr uint8_t kMaxControllers = 2;
hexapod::hal::PCA9685* controllers[kMaxControllers] = {nullptr, nullptr};
uint8_t controllerCount = 0;
SemaphoreHandle_t i2cMutex = nullptr;
std::atomic<bool> writesEnabled(true);
std::atomic<bool> outputsReleased(false);
std::atomic<uint32_t> writeFailures(0);
std::atomic<uint32_t> releaseFailures(0);

bool ensureMutex() {
  if (!i2cMutex) {
    i2cMutex = xSemaphoreCreateMutex();
  }
  return i2cMutex != nullptr;
}

}  // namespace

bool registerController(hexapod::hal::PCA9685* controller) {
  if (!controller || !ensureMutex()) {
    return false;
  }
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) != pdTRUE) {
    return false;
  }
  for (uint8_t i = 0; i < controllerCount; ++i) {
    if (controllers[i] == controller) {
      xSemaphoreGive(i2cMutex);
      return true;
    }
  }
  if (controllerCount >= kMaxControllers) {
    xSemaphoreGive(i2cMutex);
    return false;
  }
  controllers[controllerCount++] = controller;
  xSemaphoreGive(i2cMutex);
  return true;
}

bool write(hexapod::hal::PCA9685& controller,
           uint8_t channel,
           uint16_t on,
           uint16_t off) {
  if (!writesEnabled.load() || !ensureMutex()) {
    return false;
  }
  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) != pdTRUE) {
    ++writeFailures;
    return false;
  }
  const bool allowed = writesEnabled.load();
  const bool success = allowed && controller.setPWM(channel, on, off);
  if (allowed && !success) {
    ++writeFailures;
  }
  xSemaphoreGive(i2cMutex);
  return success;
}

bool releaseAll() {
  // Close the software gate before any I2C operation so no later channel write
  // can re-enable an output after the global full-off command.
  writesEnabled.store(false);
  outputsReleased.store(false);
  if (!ensureMutex() || xSemaphoreTake(i2cMutex, portMAX_DELAY) != pdTRUE) {
    ++releaseFailures;
    return false;
  }

  bool success = controllerCount > 0;
  for (uint8_t i = 0; i < controllerCount; ++i) {
    if (!controllers[i]->fullOff()) {
      ++releaseFailures;
      success = false;
    }
  }
  xSemaphoreGive(i2cMutex);
  outputsReleased.store(success);
  return success;
}

bool writeAllowed() {
  return writesEnabled.load();
}

OutputState state() {
  OutputState snapshot;
  snapshot.writeAllowed = writesEnabled.load();
  snapshot.released = outputsReleased.load();
  snapshot.writeFailures = writeFailures.load();
  snapshot.releaseFailures = releaseFailures.load();
  snapshot.controllerCount = controllerCount;
  return snapshot;
}

}  // namespace servo_output
