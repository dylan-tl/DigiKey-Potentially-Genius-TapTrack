#include "eink.h"

ThinkInk_290_Tricolor_Z94 display(
    EPD_DC, EPD_RESET, EPD_CS,
    SRAM_CS, EPD_BUSY, EPD_SPI);

void init_eink() {
    display.begin(THINKINK_TRICOLOR); 
}

void test_eink() {
    Serial.println("Banner demo");
  display.clearBuffer();
  display.setTextSize(3);
  display.setCursor((display.width() - 144) / 2, (display.height() - 24) / 2);
  display.setTextColor(EPD_BLACK);
  display.print("Digi");
  display.setTextColor(EPD_RED);
  display.print("Key");
  display.display();
}