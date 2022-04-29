#ifndef SerialClass_h
#define SerialClass_h
#include <Arduino.h>

#ifdef DEBUG_ZUMO
#define SERIAL_CLASS Serial
typedef Serial_ SerialClass;
#else
#define SERIAL_CLASS Serial1
typedef HardwareSerial SerialClass;
#endif

#endif
