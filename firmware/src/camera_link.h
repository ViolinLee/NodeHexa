#pragma once

#include <stdint.h>

namespace camera_link {

struct Snapshot {
  bool available;
  bool profilePending;
  char ip[16];
  uint16_t streamPort;
  char activeProfile[8];
  char sensor[16];
  uint32_t framesOk;
  uint32_t captureFailures;
  uint32_t encodeFailures;
  uint32_t lastStatusMs;
  bool bindingRequired;
  bool bindingVerified;
  char cameraId[24];
  char streamToken[65];
  char wifiPhase[16];
  int wifiError;
  uint16_t provisionSequence;
  char provisionResult[16];
};

void begin();
void setOffline();
void setProfilePending(bool pending);
void updateStatus(bool online,
                  const char* ip,
                  uint16_t streamPort,
                  const char* profile,
                  const char* sensor,
                  uint32_t framesOk,
                  uint32_t captureFailures,
                  uint32_t encodeFailures,
                  uint32_t nowMs);
Snapshot snapshot();
bool isValidProfile(const char* profile);
void expectBinding(bool required, const char* cameraId, const char* robotId,
                   const char* bssid, const char* session);
bool checkBinding(bool bound, const char* cameraId, const char* robotId,
                  const char* bssid, const char* session, const char* token,
                  const char* phase, int error);
void provisionSent(uint16_t sequence);
void provisionResponse(uint16_t sequence, const char* status);

}  // namespace camera_link
