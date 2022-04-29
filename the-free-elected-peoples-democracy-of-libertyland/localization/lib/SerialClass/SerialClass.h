#ifndef SerialClass_h
#define SerialClass_h
#include <Arduino.h>

#ifdef DEBUG_ZUMO
#define DATA_SERIAL_CLASS Serial
typedef Serial_ SerialClass;
#else
#define DATA_SERIAL_CLASS Serial1
typedef HardwareSerial SerialClass;
#endif

#endif
