void configureNeoPixel() {
  pixels.begin();
  pixels.clear(); // Set all pixel colors to 'off'
  //pixels.show();   // Send the updated pixel colors to the hardware.
}

void setPixelColour(uint32_t colour) {
  if (colour == white)
    pixels.setPixelColor(0, white);
  else if (colour == red)
    pixels.setPixelColor(0, red);
  else if (colour == green)
    pixels.setPixelColor(0, green);
  else if (colour == blue)
    pixels.setPixelColor(0, blue);
  else if (colour == cyan)
    pixels.setPixelColor(0, cyan);
  else if (colour == magenta)
    pixels.setPixelColor(0, magenta);
  else if (colour == yellow)
    pixels.setPixelColor(0, yellow);
  else if (colour == purple)
    pixels.setPixelColor(0, purple);
  else if (colour == orange)
    pixels.setPixelColor(0, orange);
  else if (colour == pink)
    pixels.setPixelColor(0, pink);
  else if (colour == lime)
    pixels.setPixelColor(0, lime);

  pixels.show();   // Send the updated pixel colors to the hardware
  delay(3000);
}
