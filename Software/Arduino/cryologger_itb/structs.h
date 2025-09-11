// structs.h
#pragma once
#include <stdint.h>

#ifndef SBD_MSG_SIZE
#define SBD_MSG_SIZE 34
#endif

#ifndef SBD_MT_SIZE
#define SBD_MT_SIZE 7
#endif

typedef union {
  struct __attribute__((packed)) {
    uint32_t unixtime;
    int16_t temperatureInt;
    uint16_t humidityInt;
    uint16_t pressureInt;
    int16_t pitch;
    int16_t roll;
    uint16_t heading;
    int32_t latitude;
    int32_t longitude;
    uint8_t satellites;
    uint16_t hdop;
    uint16_t voltage;
    uint16_t transmitDuration;
    uint8_t transmitStatus;
    uint16_t iterationCounter;
  };
  uint8_t bytes[SBD_MSG_SIZE];
} SBD_MO_MESSAGE;

typedef union {
  struct {
    uint8_t alarmMode;
    uint8_t alarmIntervalDay;
    uint8_t alarmIntervalHour;
    uint8_t alarmIntervalMinute;
    uint8_t transmitInterval;
    uint8_t transmitReattempts;
    uint8_t resetFlag;
  };
  uint8_t bytes[7];
} SBD_MT_MESSAGE;