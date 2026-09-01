#include "buzzer.h"

#include <Arduino.h>
#include <atomic>

#include "board_config.h"

namespace buzzer {
namespace {

static constexpr uint8_t kLedcChannel = 7;
static constexpr uint8_t kLedcResolutionBits = 10;
static constexpr uint8_t kQueueDepth = 4;
static constexpr uint32_t kAlarmFrequencyHz = 2500;
static constexpr uint32_t kAlarmOnMs = 80;
static constexpr uint32_t kAlarmGapMs = 80;
static constexpr uint32_t kAlarmCycleMs = 1000;

struct Note {
  uint16_t frequencyHz;
  uint16_t durationMs;
  uint16_t gapMs;
};

const Note kStartupNotes[] = {
  {523, 70, 35},
  {659, 70, 35},
  {784, 70, 35},
  {1047, 70, 35},
  {1319, 100, 0},
};

const Note kRemoteConnectedNotes[] = {
  {880, 90, 50},
  {1320, 120, 0},
};

QueueHandle_t cueQueue = nullptr;
TaskHandle_t buzzerTaskHandle = nullptr;
std::atomic<bool> mandatoryAlarm(false);
std::atomic<bool> initialized(false);

void silence() {
  ledcWriteTone(kLedcChannel, 0);
  ledcWrite(kLedcChannel, 0);
  digitalWrite(board::kBuzzerPin, LOW);
}

bool waitInterruptible(uint32_t durationMs) {
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    if (mandatoryAlarm.load()) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return true;
}

bool playNotes(const Note* notes, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    if (mandatoryAlarm.load()) {
      silence();
      return false;
    }
    ledcWriteTone(kLedcChannel, notes[i].frequencyHz);
    if (!waitInterruptible(notes[i].durationMs)) {
      silence();
      return false;
    }
    silence();
    if (notes[i].gapMs > 0 && !waitInterruptible(notes[i].gapMs)) {
      return false;
    }
  }
  return true;
}

void playAlarmCycle() {
  const uint32_t cycleStart = millis();
  for (uint8_t i = 0; i < 3 && mandatoryAlarm.load(); ++i) {
    ledcWriteTone(kLedcChannel, kAlarmFrequencyHz);
    vTaskDelay(pdMS_TO_TICKS(kAlarmOnMs));
    silence();
    if (i < 2) {
      vTaskDelay(pdMS_TO_TICKS(kAlarmGapMs));
    }
  }
  while (mandatoryAlarm.load() && millis() - cycleStart < kAlarmCycleMs) {
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void taskMain(void*) {
  Cue cue;
  for (;;) {
    if (mandatoryAlarm.load()) {
      xQueueReset(cueQueue);
      playAlarmCycle();
      continue;
    }
    if (xQueueReceive(cueQueue, &cue, pdMS_TO_TICKS(50)) != pdTRUE) {
      continue;
    }
    switch (cue) {
      case Cue::Startup:
        playNotes(kStartupNotes, sizeof(kStartupNotes) / sizeof(kStartupNotes[0]));
        break;
      case Cue::RemoteConnected:
        playNotes(
          kRemoteConnectedNotes,
          sizeof(kRemoteConnectedNotes) / sizeof(kRemoteConnectedNotes[0])
        );
        break;
    }
    silence();
  }
}

}  // namespace

bool begin() {
  if (initialized.load()) {
    return true;
  }
  pinMode(board::kBuzzerPin, OUTPUT);
  digitalWrite(board::kBuzzerPin, LOW);
  if (ledcSetup(kLedcChannel, 1000, kLedcResolutionBits) == 0) {
    return false;
  }
  ledcAttachPin(board::kBuzzerPin, kLedcChannel);
  silence();
  cueQueue = xQueueCreate(kQueueDepth, sizeof(Cue));
  if (!cueQueue) {
    return false;
  }
  if (xTaskCreate(taskMain, "Buzzer", 2048, nullptr, 1, &buzzerTaskHandle) != pdPASS) {
    vQueueDelete(cueQueue);
    cueQueue = nullptr;
    return false;
  }
  initialized.store(true);
  return true;
}

bool play(Cue cue) {
  if (!initialized.load() || mandatoryAlarm.load()) {
    return false;
  }
  return xQueueSend(cueQueue, &cue, 0) == pdTRUE;
}

void setMandatoryLowBatteryAlarm(bool active) {
  mandatoryAlarm.store(active);
  if (active && cueQueue) {
    xQueueReset(cueQueue);
  }
}

bool alarmActive() {
  return mandatoryAlarm.load();
}

}  // namespace buzzer
