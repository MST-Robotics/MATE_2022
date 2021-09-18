/* Zain Ul Mustafa 2017 */

#ifndef SERIALPORT_H
#define SERIALPORT_H

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>

class SerialPort
{
 private:
  HANDLE handler;
  bool connected;
  COMSTAT status;
  DWORD errors;

 public:
  SerialPort(const char *portName, const int baudRate);
  ~SerialPort();

  int readSerialPort(char *buffer, unsigned int buf_size);
  bool writeSerialPort(char *buffer, unsigned int buf_size);
  bool isConnected();
};

#endif  // SERIALPORT_H
