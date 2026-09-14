#include "camera_link.h"

#include <Arduino.h>
#include <string.h>

namespace camera_link {
namespace {

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
Snapshot state{};
char expectedRobot[24]{}, expectedBssid[18]{}, expectedSession[33]{};

void copyText(char* destination, size_t capacity, const char* source) {
  if (!source) source = "";
  strlcpy(destination, source, capacity);
}

}  // namespace

void begin() {
  portENTER_CRITICAL(&stateMux);
  memset(&state, 0, sizeof(state));
  expectedRobot[0] = expectedBssid[0] = expectedSession[0] = 0;
  state.streamPort = 81;
  strlcpy(state.activeProfile, "qvga", sizeof(state.activeProfile));
  portEXIT_CRITICAL(&stateMux);
}

void setOffline() {
  portENTER_CRITICAL(&stateMux);
  state.available = false;
  state.profilePending = false;
  state.bindingVerified = false;
  state.streamToken[0] = 0;
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

void expectBinding(bool required, const char* id, const char* robot,
                   const char* bssid, const char* session) {
  portENTER_CRITICAL(&stateMux);
  if (state.bindingRequired != required || strcmp(state.cameraId,id) ||
      strcmp(expectedRobot,robot) || strcmp(expectedBssid,bssid) || strcmp(expectedSession,session)) {
    state.available = state.bindingVerified = false;
    state.streamToken[0] = 0;
  }
  state.bindingRequired = required;
  copyText(state.cameraId,sizeof(state.cameraId),id);
  copyText(expectedRobot,sizeof(expectedRobot),robot);
  copyText(expectedBssid,sizeof(expectedBssid),bssid);
  copyText(expectedSession,sizeof(expectedSession),session);
  portEXIT_CRITICAL(&stateMux);
}

bool checkBinding(bool bound, const char* id, const char* robot,
                  const char* bssid, const char* session, const char* token,
                  const char* phase, int error) {
  bool tokenValid = strlen(token) == 64;
  for (size_t i=0; tokenValid && i<64; ++i)
    tokenValid = (token[i]>='0' && token[i]<='9') || (token[i]>='a' && token[i]<='f');
  portENTER_CRITICAL(&stateMux);
  state.bindingVerified = state.bindingRequired && bound && tokenValid &&
      !strcmp(state.cameraId,id) && !strcmp(expectedRobot,robot) &&
      !strcmp(expectedBssid,bssid) && !strcmp(expectedSession,session) &&
      !strncmp(token,expectedSession,32);
  copyText(state.streamToken,sizeof(state.streamToken),state.bindingVerified ? token : "");
  copyText(state.wifiPhase,sizeof(state.wifiPhase),phase);
  state.wifiError = error;
  const bool accepted = !state.bindingRequired || state.bindingVerified;
  if (!accepted) state.available = false;
  portEXIT_CRITICAL(&stateMux);
  return accepted;
}
void provisionSent(uint16_t sequence) {
  portENTER_CRITICAL(&stateMux);
  state.provisionSequence = sequence;
  copyText(state.provisionResult,sizeof(state.provisionResult),"sent");
  portEXIT_CRITICAL(&stateMux);
}
void provisionResponse(uint16_t sequence, const char* status) {
  portENTER_CRITICAL(&stateMux);
  if (sequence == state.provisionSequence)
    copyText(state.provisionResult,sizeof(state.provisionResult),status);
  portEXIT_CRITICAL(&stateMux);
}

}  // namespace camera_link
