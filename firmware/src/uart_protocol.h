#pragma once

#include <stddef.h>
#include <stdint.h>

namespace uart_protocol {

static constexpr size_t kMaxPayloadLength = 512;
static constexpr uint32_t kFrameTimeoutMs = 250;

enum class Format : uint8_t { Legacy, V2 };
enum class MessageType : uint8_t {
  Hello = 1,
  Request = 2,
  Response = 3,
  Event = 4,
  Heartbeat = 5,
};

struct Frame {
  Format format;
  MessageType messageType;
  uint8_t flags;
  uint16_t sequence;
  uint16_t payloadLength;
  uint8_t payload[kMaxPayloadLength + 1];
};

struct Stats {
  uint32_t rxFrames;
  uint32_t crcErrors;
  uint32_t lengthErrors;
  uint32_t timeouts;
  uint32_t legacyFrames;
};

uint16_t crc16Ccitt(const uint8_t* data, size_t length, uint16_t initial = 0xffff);
size_t encodeV2(MessageType type,
                uint8_t flags,
                uint16_t sequence,
                const uint8_t* payload,
                uint16_t payloadLength,
                uint8_t* output,
                size_t outputCapacity);

class Parser {
 public:
  Parser();
  bool feed(uint8_t byte, uint32_t nowMs, Frame& frame);
  void pollTimeout(uint32_t nowMs);
  void reset();
  const Stats& stats() const;

 private:
  enum class State : uint8_t {
    Seek,
    SeekMagic1,
    V2Header,
    V2Payload,
    V2CrcLow,
    V2CrcHigh,
    LegacyPayload,
  };

  void startLegacy(uint32_t nowMs);
  void startV2(uint32_t nowMs);
  void resetFrameState();
  void recordTimeout(uint32_t nowMs);

  State state_;
  Frame working_;
  uint8_t header_[7];
  uint8_t headerIndex_;
  uint16_t payloadIndex_;
  uint16_t receivedCrc_;
  uint32_t lastByteMs_;
  bool hasActivity_;
  Stats stats_;
};

}  // namespace uart_protocol
