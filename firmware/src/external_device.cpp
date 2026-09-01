#include "external_device.h"

#include <Arduino.h>
#include <string.h>

namespace external_device {
namespace {

constexpr uint32_t kV2OfflineTimeoutMs = 6000;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
Snapshot state{};

Type parseType(const char* device) {
  if (!device) return Type::GenericHost;
  if (strcmp(device, "camera") == 0) return Type::Camera;
  if (strcmp(device, "xiaozhi") == 0) return Type::Xiaozhi;
  return Type::GenericHost;
}

void copyText(char* destination, size_t capacity, const char* source) {
  if (!destination || capacity == 0) return;
  if (!source) source = "";
  strlcpy(destination, source, capacity);
}

}  // namespace

void begin() {
  portENTER_CRITICAL(&stateMux);
  memset(&state, 0, sizeof(state));
  state.type = Type::None;
  portEXIT_CRITICAL(&stateMux);
}

void observeHello(const char* device,
                  const char* deviceId,
                  const char* firmwareVersion,
                  uint8_t protocolVersion,
                  uint32_t nowMs) {
  Snapshot next{};
  next.type = parseType(device);
  next.online = true;
  next.lastSeenMs = nowMs;
  next.protocolVersion = protocolVersion;
  copyText(next.deviceId, sizeof(next.deviceId), deviceId);
  copyText(next.firmwareVersion, sizeof(next.firmwareVersion), firmwareVersion);
  portENTER_CRITICAL(&stateMux);
  state = next;
  portEXIT_CRITICAL(&stateMux);
}

void observeV2Frame(uint32_t nowMs) {
  portENTER_CRITICAL(&stateMux);
  if (state.type != Type::None && state.protocolVersion == 2) {
    state.lastSeenMs = nowMs;
    state.online = true;
  }
  portEXIT_CRITICAL(&stateMux);
}

void observeLegacyFrame(uint32_t nowMs) {
  portENTER_CRITICAL(&stateMux);
  state.type = Type::Xiaozhi;
  state.online = true;
  state.lastSeenMs = nowMs;
  state.protocolVersion = 1;
  strlcpy(state.deviceId, "legacy", sizeof(state.deviceId));
  state.firmwareVersion[0] = '\0';
  portEXIT_CRITICAL(&stateMux);
}

void poll(uint32_t nowMs) {
  portENTER_CRITICAL(&stateMux);
  if (state.online && state.protocolVersion == 2 &&
      nowMs - state.lastSeenMs > kV2OfflineTimeoutMs) {
    state.online = false;
  }
  portEXIT_CRITICAL(&stateMux);
}

Snapshot snapshot() {
  portENTER_CRITICAL(&stateMux);
  const Snapshot copy = state;
  portEXIT_CRITICAL(&stateMux);
  return copy;
}

const char* typeToString(Type type) {
  switch (type) {
    case Type::Camera: return "camera";
    case Type::Xiaozhi: return "xiaozhi";
    case Type::GenericHost: return "generic_host";
    case Type::None: return "none";
  }
  return "none";
}

}  // namespace external_device
