#ifndef UART_HANDLER_H_
#define UART_HANDLER_H_

#include <string>
#include <iostream>
#include <cstdio>

#include "serial/serial.h"

const uint32_t BAUD_RATE = 115200; 
const serial::Timeout timeout = serial::Timeout::simpleTimeout(100);

class UartHandler {
public:
  UartHandler(const std::string &port) : 
    uart(port, BAUD_RATE, timeout) 
    {};

  void write(const std::string &data);
  std::string read();

  inline bool isOpen() const { return uart.isOpen(); }
private:
  serial::Serial uart;
};

#endif // UART_HANDLER_H_
