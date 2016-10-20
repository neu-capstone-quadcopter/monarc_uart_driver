#include "uart_handler.h"

#include <chrono>
#include <thread>

/*
 *
 *                 Frame Layout
 * +---------------+------------+---------------+
 * | sync_word (2) | length (2) | data (length) |
 * +---------------+------------+---------------+
 *
 */

const uint16_t sync_word = 0x91D3;

void UartHandler::write(const std::string &data) {
  uint16_t len = data.length();
  uart.write((uint8_t*) &sync_word, 2);
  uart.write((uint8_t*) &len, 2);
  uart.write(data);
}

std::string UartHandler::read() {
  // Block until we see the sync word as the first two bytes
  while (true) {
    uint16_t header;
    uart.read((uint8_t*) &header, 2);
    if (header == sync_word) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  uint16_t len;
  uart.read((uint8_t*) &len, 2);
  return uart.read(len);
}
