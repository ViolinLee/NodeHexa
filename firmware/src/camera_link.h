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

}  // namespace camera_link
