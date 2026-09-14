#pragma once

#include <stdint.h>

namespace board {

#ifdef ROBOT_MODEL_NODEQUADMINI
static constexpr const char* kRobotType = "quad";
static constexpr const char* kHardwareFamily = "NodeQuadMini";
static constexpr uint8_t kLegCount = 4;
static constexpr const char* kCalibrationFilePath = "/calibration_quad.json";
#else
static constexpr const char* kRobotType = "hexa";
static constexpr const char* kHardwareFamily = "NodeHexa";
static constexpr uint8_t kLegCount = 6;
static constexpr const char* kCalibrationFilePath = "/calibration.json";
#endif

#ifndef NODEHEXA_HARDWARE_REVISION
// Existing universal packages target legacy/unknown hardware.
static constexpr uint8_t kHardwareRevision = 1;
static constexpr const char* kHardwareProfile = "legacy_or_unknown";
#else
static constexpr uint8_t kHardwareRevision = NODEHEXA_HARDWARE_REVISION;
static constexpr const char* kHardwareProfile =
    NODEHEXA_HARDWARE_REVISION == 2 ? "v2" : "legacy_or_unknown";
#endif

static constexpr uint8_t kBatteryLedPin = 25;
static constexpr uint8_t kBatteryAdcPin = 34;
static constexpr uint8_t kI2cSdaPin = 21;
static constexpr uint8_t kI2cSclPin = 22;
static constexpr uint8_t kUartRxPin = 16;
static constexpr uint8_t kUartTxPin = 17;
static constexpr uint32_t kUartBaudRate = 115200;
static constexpr uint8_t kBuzzerPin = 26;
static constexpr uint8_t kUltrasonicTriggerPin = 4;
static constexpr uint8_t kUltrasonicEchoPin = 35;

}  // namespace board
