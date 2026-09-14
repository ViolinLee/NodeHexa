#include <assert.h>
#include <string.h>

#include "uart_protocol.h"

int main() {
  const char payload[] = "{\"movementMode\":2}";
  uint8_t encoded[600];
  const size_t length = uart_protocol::encodeV2(
    uart_protocol::MessageType::Request,
    0,
    42,
    reinterpret_cast<const uint8_t*>(payload),
    strlen(payload),
    encoded,
    sizeof(encoded)
  );
  assert(length > 0);

  uart_protocol::Parser parser;
  uart_protocol::Frame frame;
  bool completed = false;
  parser.feed(0x00, 1, frame);
  parser.feed(0xA5, 2, frame);
  for (size_t i = 0; i < length; ++i) {
    completed = parser.feed(encoded[i], static_cast<uint32_t>(3 + i), frame) || completed;
  }
  assert(completed);
  assert(frame.format == uart_protocol::Format::V2);
  assert(frame.sequence == 42);
  assert(frame.payloadLength == strlen(payload));
  assert(strcmp(reinterpret_cast<const char*>(frame.payload), payload) == 0);

  encoded[length - 1] ^= 0xff;
  completed = false;
  for (size_t i = 0; i < length; ++i) {
    completed = parser.feed(encoded[i], static_cast<uint32_t>(100 + i), frame) || completed;
  }
  assert(!completed);
  assert(parser.stats().crcErrors == 1);

  const char legacy[] = "${\"speed\":0.5}\n";
  for (size_t i = 0; i < strlen(legacy); ++i) {
    completed = parser.feed(legacy[i], static_cast<uint32_t>(200 + i), frame);
  }
  assert(completed);
  assert(frame.format == uart_protocol::Format::Legacy);
  assert(strcmp(reinterpret_cast<const char*>(frame.payload), "{\"speed\":0.5}") == 0);

  // A new '$' restarts a truncated legacy frame instead of contaminating JSON.
  uart_protocol::Parser legacyResyncParser;
  const char legacyResync[] = "$truncated${\"speed\":0.25}\n";
  completed = false;
  for (size_t i = 0; i < strlen(legacyResync); ++i) {
    completed = legacyResyncParser.feed(
      legacyResync[i], static_cast<uint32_t>(250 + i), frame) || completed;
  }
  assert(completed);
  assert(strcmp(reinterpret_cast<const char*>(frame.payload), "{\"speed\":0.25}") == 0);

  // A partial magic prefix followed by noise must not create a false timeout.
  uart_protocol::Parser noiseParser;
  noiseParser.feed(0xA5, 270, frame);
  noiseParser.feed(0x00, 271, frame);
  noiseParser.pollTimeout(600);
  assert(noiseParser.stats().timeouts == 0);

  parser.feed('$', 300, frame);
  parser.feed('{', 301, frame);
  parser.pollTimeout(600);
  assert(parser.stats().timeouts == 1);

  // A '$' byte is valid inside a V2 payload and must not switch to legacy mode.
  const char dollarPayload[] = "{\"ssid\":\"cost$center\"}";
  const size_t dollarLength = uart_protocol::encodeV2(
    uart_protocol::MessageType::Request, 0, 43,
    reinterpret_cast<const uint8_t*>(dollarPayload), strlen(dollarPayload),
    encoded, sizeof(encoded));
  completed = false;
  for (size_t i = 0; i < dollarLength; ++i) {
    completed = parser.feed(encoded[i], static_cast<uint32_t>(700 + i), frame) || completed;
  }
  assert(completed);
  assert(strcmp(reinterpret_cast<const char*>(frame.payload), dollarPayload) == 0);

  // Oversized headers are rejected, and the next valid frame still decodes.
  const uint8_t oversized[] = {0xA5, 0x4E, 0x02, 0x02, 0x00, 0x01, 0x00, 0x01, 0x02};
  for (size_t i = 0; i < sizeof(oversized); ++i) {
    assert(!parser.feed(oversized[i], static_cast<uint32_t>(800 + i), frame));
  }
  assert(parser.stats().lengthErrors == 1);

  const char compactPayload[] = "{}";
  const size_t compactLength = uart_protocol::encodeV2(
    uart_protocol::MessageType::Heartbeat, 0, 44,
    reinterpret_cast<const uint8_t*>(compactPayload), strlen(compactPayload),
    encoded, sizeof(encoded));
  completed = false;
  for (size_t i = 0; i < compactLength; ++i) {
    completed = parser.feed(encoded[i], static_cast<uint32_t>(900 + i), frame) || completed;
  }
  assert(completed && frame.sequence == 44);

  // Truncation times out, then arbitrary noise followed by continuous valid
  // frames must recover without reset or data loss.
  for (size_t i = 0; i < compactLength / 2; ++i) {
    parser.feed(encoded[i], static_cast<uint32_t>(1000 + i), frame);
  }
  parser.pollTimeout(1400);
  assert(parser.stats().timeouts == 2);
  const uint8_t noise[] = {0x00, 0xff, 0xA5, 0x00, '\n', 0x7f};
  for (size_t i = 0; i < sizeof(noise); ++i) {
    parser.feed(noise[i], static_cast<uint32_t>(1500 + i), frame);
  }
  unsigned decoded = 0;
  for (unsigned repeat = 0; repeat < 32; ++repeat) {
    for (size_t i = 0; i < compactLength; ++i) {
      if (parser.feed(encoded[i], static_cast<uint32_t>(1600 + repeat * compactLength + i), frame)) {
        ++decoded;
      }
    }
  }
  assert(decoded == 32);
  assert(parser.stats().rxFrames == 36);
  return 0;
}
