/*
  imu_calibration.h

  Per-unit magnetometer calibration constants for Cryologger ITBs.
  Uses the serial number defined in the main tab to select the correct
  calibration. If no match is found, the code falls back to identity
  (no correction applied), which is the correct starting state for
  in-field self-calibration.
*/

#pragma once
#include <Arduino.h>

struct MagCal {
  const char* serialNumber;  // e.g. "ITB_26_TST"
  float B[3];                // Hard-iron bias (µT)
  float Ainv[3][3];          // Inverse soft-iron matrix
  float p[3];                // Forward vector for this beacon
};

// ----------------------------------------------------------------------------
// Calibration table
// Replace placeholder entry with real Magneto output for each beacon.
// ----------------------------------------------------------------------------
static const MagCal kCalTable[] = {

  { "ITB_PLACEHOLDER",
    { 0.0f, 0.0f, 0.0f },          // Hard-iron bias (µT) — uncalibrated
    { { 1.0f, 0.0f, 0.0f },        // Inverse soft-iron matrix — identity (no correction)
      { 0.0f, 1.0f, 0.0f },
      { 0.0f, 0.0f, 1.0f } },
    { 1.0f, 0.0f, 0.0f } },        // Forward vector — align to X+
};

static const size_t kCalTableCount = sizeof(kCalTable) / sizeof(kCalTable[0]);

static inline const MagCal* getSelectedCal() {
  for (size_t i = 0; i < kCalTableCount; ++i) {
    if (strcmp(kCalTable[i].serialNumber, SERIAL_NUMBER) == 0) return &kCalTable[i];
  }
  return nullptr;  // Fallback: no calibration found — identity used in 07_imu.ino
}