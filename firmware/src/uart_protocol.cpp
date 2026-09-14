#include "uart_protocol.h"

#include <string.h>

namespace uart_protocol {
namespace {

static constexpr uint8_t kMagic0 = 0xA5;
static constexpr uint8_t kMagic1 = 0x4E;
static constexpr uint8_t kVersion = 0x02;

}  // namespace

uint16_t crc16Ccitt(const uint8_t* data, size_t length, uint16_t initial) {
  uint16_t crc = initial;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

size_t encodeV2(MessageType type,
                uint8_t flags,
                uint16_t sequence,
                const uint8_t* payload,
                uint16_t payloadLength,
                uint8_t* output,
                size_t outputCapacity) {
  const size_t encodedLength = 11 + payloadLength;
  if (!output || payloadLength > kMaxPayloadLength || outputCapacity < encodedLength ||
      (payloadLength > 0 && !payload)) {
    return 0;
  }
  output[0] = kMagic0;
  output[1] = kMagic1;
  output[2] = kVersion;
  output[3] = static_cast<uint8_t>(type);
  output[4] = flags;
  output[5] = static_cast<uint8_t>(sequence & 0xff);
  output[6] = static_cast<uint8_t>(sequence >> 8);
  output[7] = static_cast<uint8_t>(payloadLength & 0xff);
  output[8] = static_cast<uint8_t>(payloadLength >> 8);
  if (payloadLength > 0) {
    memcpy(output + 9, payload, payloadLength);
  }
  const uint16_t crc = crc16Ccitt(output + 2, 7 + payloadLength);
  output[9 + payloadLength] = static_cast<uint8_t>(crc & 0xff);
  output[10 + payloadLength] = static_cast<uint8_t>(crc >> 8);
  return encodedLength;
}

Parser::Parser() {
  memset(&stats_, 0, sizeof(stats_));
  resetFrameState();
}

void Parser::resetFrameState() {
  state_ = State::Seek;
  headerIndex_ = 0;
  payloadIndex_ = 0;
  receivedCrc_ = 0;
  lastByteMs_ = 0;
  hasActivity_ = false;
  memset(&working_, 0, sizeof(working_));
}

void Parser::reset() {
  resetFrameState();
}

void Parser::startLegacy(uint32_t nowMs) {
  resetFrameState();
  state_ = State::LegacyPayload;
  working_.format = Format::Legacy;
  working_.messageType = MessageType::Request;
  lastByteMs_ = nowMs;
  hasActivity_ = true;
}

void Parser::startV2(uint32_t nowMs) {
  resetFrameState();
  state_ = State::V2Header;
  working_.format = Format::V2;
  lastByteMs_ = nowMs;
  hasActivity_ = true;
}

void Parser::recordTimeout(uint32_t nowMs) {
  if (hasActivity_ && nowMs - lastByteMs_ > kFrameTimeoutMs) {
    ++stats_.timeouts;
    resetFrameState();
  }
}

void Parser::pollTimeout(uint32_t nowMs) {
  recordTimeout(nowMs);
}

bool Parser::feed(uint8_t byte, uint32_t nowMs, Frame& frame) {
  recordTimeout(nowMs);
  // Legacy '$' is only a start marker while seeking. Inside a V2 frame it is
  // ordinary payload/header data and must never desynchronize the parser.
  if ((state_ == State::Seek || state_ == State::SeekMagic1) && byte == '$') {
    startLegacy(nowMs);
    return false;
  }
  lastByteMs_ = nowMs;

  switch (state_) {
    case State::Seek:
      if (byte == kMagic0) {
        state_ = State::SeekMagic1;
        hasActivity_ = true;
      }
      return false;

    case State::SeekMagic1:
      if (byte == kMagic1) {
        startV2(nowMs);
      } else if (byte == kMagic0) {
        // Keep the second possible magic prefix and refresh its timeout.
        state_ = State::SeekMagic1;
      } else {
        // A mismatched magic prefix is ordinary noise, not an active frame.
        resetFrameState();
      }
      return false;

    case State::LegacyPayload:
      if (byte == '$') {
        // A fresh legacy marker supersedes a truncated legacy message.
        startLegacy(nowMs);
        return false;
      }
      if (byte == '\r' || byte == '\n') {
        if (payloadIndex_ == 0) {
          resetFrameState();
          return false;
        }
        working_.payloadLength = payloadIndex_;
        working_.payload[payloadIndex_] = 0;
        frame = working_;
        ++stats_.rxFrames;
        ++stats_.legacyFrames;
        resetFrameState();
        return true;
      }
      if (payloadIndex_ >= kMaxPayloadLength) {
        ++stats_.lengthErrors;
        resetFrameState();
        return false;
      }
      working_.payload[payloadIndex_++] = byte;
      return false;

    case State::V2Header:
      header_[headerIndex_++] = byte;
      if (headerIndex_ < sizeof(header_)) {
        return false;
      }
      if (header_[0] != kVersion) {
        resetFrameState();
        return false;
      }
      working_.messageType = static_cast<MessageType>(header_[1]);
      working_.flags = header_[2];
      working_.sequence = static_cast<uint16_t>(header_[3]) |
                          (static_cast<uint16_t>(header_[4]) << 8);
      working_.payloadLength = static_cast<uint16_t>(header_[5]) |
                               (static_cast<uint16_t>(header_[6]) << 8);
      if (working_.payloadLength > kMaxPayloadLength) {
        ++stats_.lengthErrors;
        resetFrameState();
        return false;
      }
      state_ = working_.payloadLength == 0 ? State::V2CrcLow : State::V2Payload;
      return false;

    case State::V2Payload:
      working_.payload[payloadIndex_++] = byte;
      if (payloadIndex_ >= working_.payloadLength) {
        working_.payload[payloadIndex_] = 0;
        state_ = State::V2CrcLow;
      }
      return false;

    case State::V2CrcLow:
      receivedCrc_ = byte;
      state_ = State::V2CrcHigh;
      return false;

    case State::V2CrcHigh: {
      receivedCrc_ |= static_cast<uint16_t>(byte) << 8;
      uint16_t expected = crc16Ccitt(header_, sizeof(header_));
      expected = crc16Ccitt(working_.payload, working_.payloadLength, expected);
      if (receivedCrc_ != expected) {
        ++stats_.crcErrors;
        resetFrameState();
        return false;
      }
      working_.payload[working_.payloadLength] = 0;
      frame = working_;
      ++stats_.rxFrames;
      resetFrameState();
      return true;
    }
  }
  resetFrameState();
  return false;
}

const Stats& Parser::stats() const {
  return stats_;
}

}  // namespace uart_protocol
