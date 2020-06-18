#pragma once
#include <ModbusRTU.h>

#define BSIZE 1024

uint8_t buf1[BSIZE];
uint8_t buf2[BSIZE];

StreamBuf S1(buf1, BSIZE);
StreamBuf S2(buf2, BSIZE);
DuplexBuf D1(&S1, &S2);
DuplexBuf D2(&S2, &S1);

ModbusRTU master;
ModbusRTU slave;

bool result;
uint8_t code ;

bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  //Serial.printf_P(" 0x%02X ", event);
  //if (event == 0x00) {
  code = event;
  result = true;
  return true;
}

uint8_t wait() {
  result = false;
  code = 0;
  while (!result) {
    master.task();
    slave.task();
    yield();
  }
  Serial.printf_P(" 0x%02X", code);
  return code;
}
