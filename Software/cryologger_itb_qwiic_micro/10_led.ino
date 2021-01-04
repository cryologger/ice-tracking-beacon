// Configure WS2812B RGB LED
void configureLed()
{
  led.begin();
  led.clear(); // Set all pixel colors to 'off'
  led.show(); // Send the updated pixel colors to the hardware
}

// Set WS2812B RGB LED color
void setLedColour(uint32_t colour)
{
  if (colour == white)
    led.setPixelColor(0, white);
  else if (colour == red)
    led.setPixelColor(0, red);
  else if (colour == green)
    led.setPixelColor(0, green);
  else if (colour == blue)
    led.setPixelColor(0, blue);
  else if (colour == cyan)
    led.setPixelColor(0, cyan);
  else if (colour == magenta)
    led.setPixelColor(0, magenta);
  else if (colour == yellow)
    led.setPixelColor(0, yellow);
  else if (colour == purple)
    led.setPixelColor(0, purple);
  else if (colour == orange)
    led.setPixelColor(0, orange);
  else if (colour == pink)
    led.setPixelColor(0, pink);
  else if (colour == lime)
    led.setPixelColor(0, lime);
  else if (colour == off)
    led.clear(); // Set all pixel colors to 'off'

  led.show();   // Send the updated pixel colors to the hardware

  // Non-blocking delay
  unsigned long currentMillis = millis();
  while (millis() - currentMillis < ledDelay) {
    // delay
  }
}
