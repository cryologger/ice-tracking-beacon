/*
   Code to test battery voltage 10/1 MOhm resistor divider
*/
#define PIN_VBAT            A0
#define PIN_SENSOR_EN       A3
#define PIN_IMU_EN          A4
#define PIN_GNSS_EN         A5
#define PIN_LED             5
#define PIN_IRIDIUM_EN      6
#define PIN_IRIDIUM_RX      10 // Pin 1 RXD (Yellow)
#define PIN_IRIDIUM_TX      11 // Pin 6 TXD (Orange)
#define PIN_IRIDIUM_SLEEP   12 // Pin 7 OnOff (Grey)

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_IRIDIUM_EN, OUTPUT);
  pinMode(PIN_GNSS_EN, OUTPUT);
  pinMode(PIN_SENSOR_EN, OUTPUT);
  pinMode(PIN_IMU_EN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_SENSOR_EN, LOW);   // Disable power to sensors
  digitalWrite(PIN_GNSS_EN, HIGH);     // Disable power to GNSS
  digitalWrite(PIN_IMU_EN, LOW);      // Disable power to IMU
  digitalWrite(PIN_IRIDIUM_EN, LOW);  // Disable power to Iridium 9603
  
  Serial.begin(115200);
  while (!Serial);

  // Configure ADC
  ADC->CTRLA.bit.ENABLE = 0;                      // Disable ADC
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |   // Divide Clock ADC GCLK by 512 (48MHz/512 = 93.7kHz)
                   ADC_CTRLB_RESSEL_16BIT;        // Set ADC resolution to 12-bit
  while (ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(64);   // Set Sampling Time Length (341.33 us)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_512 |  // Configure multisampling
                     ADC_AVGCTRL_ADJRES(4);       // Configure averaging
  while (ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization
  ADC->CTRLA.bit.ENABLE = 1;                      // Enable ADC
  while (ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  // Apply ADC gain and offset error calibration correction
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(28);
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(2101);
  ADC->CTRLB.bit.CORREN = true;
  while (ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization
}

void loop()
{
  (void)analogRead(A0);
  int sensorValue = analogRead(A0);

  float voltage1 = sensorValue * (3.3 / 4095.0);
  float voltage2 = sensorValue * ((10000000.0 + 1000000.0) / 1000000.0); // Multiply back 1 MOhm / (10 MOhm + 1 MOhm)
  voltage2 *= 3.3;   // Multiply by 3.3V reference voltage
  voltage2 /= 4096;  // Convert to voltage
  Serial.print(F("sensorValue: ")); Serial.print(sensorValue); Serial.print(F(",")); Serial.print(voltage1, 4); Serial.print(F(",")); Serial.println(voltage2, 4);

  delay(1000);
}
