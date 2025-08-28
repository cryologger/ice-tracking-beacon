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
    { -39.553345, 34.629121, -25.846887 },
    { { 22.951053, 1.140622, -0.069905 },
      { 1.140622, 21.816512, -0.046289 },
      { -0.069905, -0.046289, 22.323099 } },
    { 1.0, 0.0, 0.0 } },
  { "ITB_25_039",
    { 0.0f, 0.0f, 0.0f },
    { { 1.0f, 0.0f, 0.0f },
      { 0.0f, 1.0f, 0.0f },
      { 0.0f, 0.0f, 1.0f } },
    { 1.0f, 0.0f, 0.0f } },
  { "ITB_25_040",
    { 0.0f, 0.0f, 0.0f },
    { { 1.0f, 0.0f, 0.0f },
      { 0.0f, 1.0f, 0.0f },
      { 0.0f, 0.0f, 1.0f } },
    { 1.0f, 0.0f, 0.0f } },
  { "ITB_25_041",
    { 0.0f, 0.0f, 0.0f },
    { { 1.0f, 0.0f, 0.0f },
      { 0.0f, 1.0f, 0.0f },
      { 0.0f, 0.0f, 1.0f } },
    { 1.0f, 0.0f, 0.0f } }
};

static const size_t kCalTableCount = sizeof(kCalTable) / sizeof(kCalTable[0]);

static inline const MagCal* getSelectedCal() {
  for (size_t i = 0; i < kCalTableCount; ++i) {
    if (strcmp(kCalTable[i].uid, UID) == 0) return &kCalTable[i];
  }
  return nullptr;  // Fallback: no calibration found
}
