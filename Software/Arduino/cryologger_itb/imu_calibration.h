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
    { -39.139259, 33.155673, -27.560482 },
    { { 23.425088, 1.196800, -0.105585 },
      { 1.196800, 22.511785, -0.119593 },
      { -0.105585, -0.119593, 23.032977 } },
    { 1.0, 0.0, 0.0 } },
  { "ITB_25_039",
    { 19.789020, -18.329800, -14.639069 },
    { { 23.088669, 0.542799, -0.405731 },
      { 0.542799, 22.563486, -0.257519 },
      { -0.405731, -0.257519, 23.132156 } },
    { 1.0, 0.0, 0.0 } },
  { "ITB_25_040",
    { -48.000064, 33.492765, -46.684310 },
    { { 23.469375, 1.240727, -0.202023 },
      { 1.240727, 22.350662, -0.178700 },
      { -0.202023, -0.178700, 22.925058 } },
    { 1.0, 0.0, 0.0 } },
  { "ITB_25_041",
    { -26.244750, 35.755771, -12.474339 },
    { { 22.548286, 0.964679, -0.248699 },
      { 0.964679, 22.142424, -0.135316 },
      { -0.248699, -0.135316, 23.016529 } },
    { 1.0, 0.0, 0.0 } }
};

static const size_t kCalTableCount = sizeof(kCalTable) / sizeof(kCalTable[0]);

static inline const MagCal* getSelectedCal() {
  for (size_t i = 0; i < kCalTableCount; ++i) {
    if (strcmp(kCalTable[i].uid, UID) == 0) return &kCalTable[i];
  }
  return nullptr;  // Fallback: no calibration found
}
