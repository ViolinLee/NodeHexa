#include "camera_link.h"

#include <Arduino.h>
#include <string.h>

namespace camera_link {
namespace {

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
Snapshot state{};

void copyText(char* destination, size_t capacity, const char* source) {
  if (!source) source = "";
  strlcpy(destination, source, capacity);
}

}  // namespace

void begin() {
  portENTER_CRITICAL(&stateMux);
  memset(&state, 0, sizeof(state));
  state.streamPort = 81;
  strlcpy(state.activeProfile, "qvga", sizeof(state.activeProfile));
  portEXIT_CRITICAL(&stateMux);
}

void setOffline() {
  portENTER_CRITICAL(&stateMux);
  state.available = false;
  state.profilePending = false;
  portEXIT_CRITICAL(&stateMux);
}

void setProfilePending(bool pending) {
  portENTER_CRITICAL(&stateMux);
  state.profilePending = pending;
  portEXIT_CRITICAL(&stateMux);
}

void updateStatus(bool online,
                  const char* ip,
                  uint16_t streamPort,
                  const char* profile,
                  const char* sensor,
                  uint32_t framesOk,
                  uint32_t captureFailures,
                  uint32_t encodeFailures,
                  uint32_t nowMs) {
  portENTER_CRITICAL(&stateMux);
  state.available = online && ip && ip[0] != '\0';
  state.profilePending = false;
  copyText(state.ip, sizeof(state.ip), ip);
  state.streamPort = streamPort == 0 ? 81 : streamPort;
  if (isValidProfile(profile)) copyText(state.activeProfile, sizeof(state.activeProfile), profile);
  copyText(state.sensor, sizeof(state.sensor), sensor);
  state.framesOk = framesOk;
  state.captureFailures = captureFailures;
  state.encodeFailures = encodeFailures;
  state.lastStatusMs = nowMs;
  portEXIT_CRITICAL(&stateMux);
}

Snapshot snapshot() {
  portENTER_CRITICAL(&stateMux);
  const Snapshot copy = state;
  portEXIT_CRITICAL(&stateMux);
  return copy;
}

bool isValidProfile(const char* profile) {
  return profile && (strcmp(profile, "qvga") == 0 || strcmp(profile, "vga") == 0);
}

}  // namespace camera_link
