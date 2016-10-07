#include "uart_handler.h"

#include <arpa/inet.h>

void UartHandler::write(const std::string &data) {
  uint16_t len = htons(data.length());
  uart.write((uint8_t*) &len, 2);
  uart.write(data);
}

std::string UartHandler::read() {
  uint16_t len;
  uart.read((uint8_t*) &len, 2);
  return uart.read(ntohs(len)).substr(0, len);
}
