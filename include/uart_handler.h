#ifndef UART_HANDLER_H_
#define UART_HANDLER_H_

#include <string>
#include <iostream>
#include <cstdio>

#include "serial/serial.h"

const std::string     default_uart_port = "/dev/ttyHS1";
const int             default_baud_rate = 115200;
const serial::Timeout default_timeout = serial::Timeout::simpleTimeout(50);

class UartHandler {
public:
  UartHandler(const std::string &port, const uint32_t baud_rate) :
    uart(port, baud_rate, default_timeout)
    {};

  void write(const std::string &data);
  std::string read();

  inline bool isOpen() const { return uart.isOpen(); }
private:
  serial::Serial uart;
};

#endif // UART_HANDLER_H_
