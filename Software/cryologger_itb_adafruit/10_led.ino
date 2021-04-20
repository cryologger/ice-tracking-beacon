// Configure WS2812B RGB LED
void configureLed()
{
  FastLED.addLeds<WS2812B, PIN_LED, GRB>(led, 1);
  FastLED.setBrightness(16);
}

// Set WS2812B RGB LED color
// https://forum.arduino.cc/index.php?topic=647732.0
void setLedColour(CRGB colour)
{
  led[0] = colour;
  FastLED.show();
  myDelay(2000);
}
