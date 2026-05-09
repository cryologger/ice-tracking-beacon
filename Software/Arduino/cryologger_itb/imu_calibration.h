/*
  imu_calibration.h

  Per-unit magnetometer calibration constants for Cryologger ITBs.
  Uses the serial number defined in the main tab to select the correct 
  calibration.
*/

#pragma once
#include <Arduino.h>

struct MagCal {
  const char* serialNumber;  // e.g. "ITB_26_042"
  float B[3];                // Hard-iron bias (µT)
  float Ainv[3][3];          // Inverse soft-iron matrix
  float p[3];                // Forward vector for this beacon
};

// ---- REPLACE THESE VALUES WITH REAL CALIBRATIONS ----
static const MagCal kCalTable[] = {

  { "ITB_25_042",
    { -26.351757, 43.980509, -33.265206 },
    { { 23.705803, 0.990113, 1.651733 },
      { 0.990113, 20.786501, -0.012713 },
      { 1.651733, -0.012713, 16.145779 } },
    { 1.0, 0.0, 0.0 } }
};

static const size_t kCalTableCount = sizeof(kCalTable) / sizeof(kCalTable[0]);

static inline const MagCal* getSelectedCal() {
  for (size_t i = 0; i < kCalTableCount; ++i) {
    if (strcmp(kCalTable[i].serialNumber, SERIAL_NUMBER) == 0) return &kCalTable[i];
  }
  return nullptr;  // Fallback: no calibration found
}
