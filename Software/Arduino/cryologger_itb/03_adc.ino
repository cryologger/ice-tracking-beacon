/*
  Analog-to-digital Converter (ADC) Module

  This module configures the SAMD21 ADC, provides a helper function to map raw 
  ADC values to a floating-point range, and includes a calibration function for
  debugging or accuracy checks.
*/

// ----------------------------------------------------------------------------
// Configures the SAMD21 ADC with 12-bit resolution, clock prescaler, sampling
// time, averaging, and gain/offset corrections. For more details, see:
// https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/
//
// Sample time: 174,762.67 us
// Conversion time: 174,848 us
// Max input impedance: 10,808,452 Ohms
// ----------------------------------------------------------------------------
void configureAdc() {
  // Disable ADC before configuration
  ADC->CTRLA.bit.ENABLE = 0;

  // Clock prescaler, resolution, etc.
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |  // Divide Clock ADC GCLK by 512 (48MHz/512 = 93.7kHz).
                   ADC_CTRLB_RESSEL_16BIT;       // Set ADC resolution to 12-bit.
  while (ADC->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Sampling time length (341.33 us)
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(64);
  // Multisampling (512 samples), average adjusts resolution by 4 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_512 | ADC_AVGCTRL_ADJRES(4);
  while (ADC->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Re-enable ADC
  ADC->CTRLA.bit.ENABLE = 1;
  while (ADC->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Apply gain/offset error calibration
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(0);
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(2048);
  ADC->CTRLB.bit.CORREN = true;
  while (ADC->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization
}

// ----------------------------------------------------------------------------
// Maps raw ADC values to floats.
// ----------------------------------------------------------------------------
float mapFloat(float x, float in_min, float in_max,
               float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ----------------------------------------------------------------------------
// Reads raw ADC values from PIN_VBAT and computes two voltages:
// voltage1 — raw ADC pin voltage (direct 3.3 V / 4096 scale)
// voltage2 — actual battery voltage (scaled through 10 MΩ + 1 MΩ divider)
// ----------------------------------------------------------------------------
void calibrateAdc() {
  float sensorValue = analogRead(PIN_VBAT);

  // Direct scaling: raw ADC pin voltage (0–3.3 V)
  float voltage1 = sensorValue * (3.3f / 4096.0f);

  // Voltage divider-based scaling: actual battery voltage
  float voltage2 = sensorValue * ((10.0f + 1.0f) / 1.0f);  // Factor for 10 MΩ + 1 MΩ
  voltage2 *= 3.3f;                                        // 3.3 V reference
  voltage2 /= 4096.0f;                                     // Convert to voltage

  DEBUG_PRINTLN(F("[ADC] Info: sensorValue, voltage1, voltage2"));
  DEBUG_PRINT(F("[ADC] Info:"));
  DEBUG_PRINT(sensorValue);
  DEBUG_PRINT(F(","));
  DEBUG_PRINT_DEC(voltage1, 4);
  DEBUG_PRINT(F(","));
  DEBUG_PRINTLN_DEC(voltage2, 4);
}