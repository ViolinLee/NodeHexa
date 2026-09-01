#pragma once

#include <stdint.h>

namespace external_device {

enum class Type : uint8_t { None, Camera, Xiaozhi, GenericHost };

struct Snapshot {
  Type type;
  bool online;
  uint32_t lastSeenMs;
  uint8_t protocolVersion;
  char deviceId[32];
  char firmwareVersion[24];
};

void begin();
void observeHello(const char* device,
                  const char* deviceId,
                  const char* firmwareVersion,
                  uint8_t protocolVersion,
                  uint32_t nowMs);
void observeV2Frame(uint32_t nowMs);
void observeLegacyFrame(uint32_t nowMs);
void poll(uint32_t nowMs);
Snapshot snapshot();
const char* typeToString(Type type);

}  // namespace external_device
