/*
  imu_calibration.h

  Per-unit magnetometer calibration constants for Cryologger ITBs.
  Uses the UID defined in the main tab to select the correct calibration.
*/

#pragma once
#include <Arduino.h>

struct MagCal {
  const char* uid;   // e.g. "ITB_25_038"
  float B[3];        // Hard-iron bias (µT)
  float Ainv[3][3];  // Inverse soft-iron matrix
  float p[3];        // Forward vector for this beacon
};

// ---- REPLACE THESE VALUES WITH REAL CALIBRATIONS ----
static const MagCal kCalTable[] = {
  { "ITB_25_038",
    { -38.432981, 41.729639, -49.201732 },
    { { 25.084557, 1.277949, 1.985049 },
      { 1.277949, 21.788606, -0.106281 },
      { 1.985049, -0.106281, 16.689121 } },
    { 1.0, 0.0, 0.0 } },
  { "ITB_25_039",
    { 20.374425, -9.999212, -35.924206 },
    { { 24.270325, 0.708608, 1.823561 },
      { 0.708608, 20.930094, 0.137676 },
      { 1.823561, 0.137676, 16.218993 } },
    { 1.0, 0.0, 0.0 } },
  { "ITB_25_040",
    { -47.436639, 41.565085, -69.007085 },
    { { 25.125385, 1.216390, 1.614394 },
      { 1.216390, 21.172139, -0.108664 },
      { 1.614394, -0.108664, 16.093389 } },
    { 1.0, 0.0, 0.0 } },
  { "ITB_25_041",
    { -26.351757, 43.980509, -33.265206 },
    { { 23.705803, 0.990113, 1.651733 },
      { 0.990113, 20.786501, -0.012713 },
      { 1.651733, -0.012713, 16.145779 } },
    { 1.0, 0.0, 0.0 } }
};

static const size_t kCalTableCount = sizeof(kCalTable) / sizeof(kCalTable[0]);

static inline const MagCal* getSelectedCal() {
  for (size_t i = 0; i < kCalTableCount; ++i) {
    if (strcmp(kCalTable[i].uid, UID) == 0) return &kCalTable[i];
  }
  return nullptr;  // Fallback: no calibration found
}
