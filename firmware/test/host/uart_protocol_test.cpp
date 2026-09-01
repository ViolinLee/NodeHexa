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

  parser.feed('$', 300, frame);
  parser.feed('{', 301, frame);
  parser.pollTimeout(600);
  assert(parser.stats().timeouts == 1);
  return 0;
}
