#include "ultrasonic_sensor.h"

#include <Arduino.h>

#include "board_config.h"

namespace ultrasonic {
namespace {

constexpr uint32_t kEchoHardTimeoutUs = 30000;
constexpr uint32_t kMinimumValidPulseUs = 100;
constexpr uint32_t kMinimumRequestTimeoutMs = 31;
constexpr uint32_t kMaximumRequestTimeoutMs = 40;

struct Request {
  uint32_t timeoutMs;
};

SemaphoreHandle_t stateMutex = nullptr;
SemaphoreHandle_t measurementMutex = nullptr;
QueueHandle_t requestQueue = nullptr;
Reading state{};

uint32_t clampTimeout(uint32_t timeoutMs) {
  if (timeoutMs < kMinimumRequestTimeoutMs) return kMinimumRequestTimeoutMs;
  if (timeoutMs > kMaximumRequestTimeoutMs) return kMaximumRequestTimeoutMs;
  return timeoutMs;
}

void publish(const Reading& reading) {
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    const bool pending = state.pending;
    state = reading;
    state.pending = pending;
    xSemaphoreGive(stateMutex);
  }
}

void setPending(bool pending) {
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    state.pending = pending;
    xSemaphoreGive(stateMutex);
  }
}

void worker(void*) {
  Request request{};
  while (true) {
    if (xQueueReceive(requestQueue, &request, portMAX_DELAY) != pdTRUE) continue;
    Reading reading{};
    measureOnce(reading, request.timeoutMs);
    setPending(false);
  }
}

}  // namespace

bool begin() {
  pinMode(board::kUltrasonicTriggerPin, OUTPUT);
  digitalWrite(board::kUltrasonicTriggerPin, LOW);
  pinMode(board::kUltrasonicEchoPin, INPUT);
  stateMutex = xSemaphoreCreateMutex();
  measurementMutex = xSemaphoreCreateMutex();
  requestQueue = xQueueCreate(1, sizeof(Request));
  if (!stateMutex || !measurementMutex || !requestQueue) return false;
  return xTaskCreate(worker, "Ultrasonic", 3072, nullptr, 1, nullptr) == pdPASS;
}

bool requestMeasurement(uint32_t timeoutMs) {
  if (!requestQueue || !stateMutex) return false;
  bool pending = false;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    pending = state.pending;
    if (!pending) state.pending = true;
    xSemaphoreGive(stateMutex);
  } else {
    return false;
  }
  if (pending) return false;
  Request request{clampTimeout(timeoutMs)};
  if (xQueueSend(requestQueue, &request, 0) != pdTRUE) {
    setPending(false);
    return false;
  }
  return true;
}

bool measureOnce(Reading& out, uint32_t timeoutMs) {
  if (!measurementMutex ||
      xSemaphoreTake(measurementMutex, pdMS_TO_TICKS(clampTimeout(timeoutMs))) != pdTRUE) {
    return false;
  }

  digitalWrite(board::kUltrasonicTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(board::kUltrasonicTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(board::kUltrasonicTriggerPin, LOW);

  const uint32_t pulseWidthUs = pulseIn(board::kUltrasonicEchoPin, HIGH, kEchoHardTimeoutUs);
  Reading next = latest();
  next.pending = true;
  ++next.sampleCount;
  next.measuredAtMs = millis();
  next.ageMs = 0;
  next.pulseWidthUs = pulseWidthUs;
  next.distanceMm = 0;
  next.valid = false;
  if (pulseWidthUs == 0) {
    ++next.timeoutCount;
  } else if (pulseWidthUs < kMinimumValidPulseUs) {
    ++next.errorCount;
  } else {
    next.valid = true;
    next.distanceMm = static_cast<uint32_t>((static_cast<uint64_t>(pulseWidthUs) * 343u) / 2000u);
  }
  publish(next);
  out = next;
  xSemaphoreGive(measurementMutex);
  return true;
}

Reading latest() {
  Reading copy{};
  if (!stateMutex) return copy;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    copy = state;
    xSemaphoreGive(stateMutex);
  }
  copy.ageMs = copy.measuredAtMs == 0 ? 0 : millis() - copy.measuredAtMs;
  return copy;
}

bool online(uint32_t maxAgeMs) {
  const Reading reading = latest();
  return reading.valid && reading.measuredAtMs != 0 && reading.ageMs <= maxAgeMs;
}

}  // namespace ultrasonic
