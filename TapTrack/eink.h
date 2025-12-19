#ifndef EINK_H
#define EINK_H

#include <Arduino.h>
#include "Adafruit_ThinkInk.h"

#define EPD_DC 10
#define EPD_CS 9
//#define EPD_BUSY A4 // can set to -1 to not use a pin (will wait a fixed delay)
#define EPD_BUSY -1
#define SRAM_CS 6
#define EPD_RESET -1  // can set to -1 and share with microcontroller Reset!
#define EPD_SPI &SPI // primary SPI

extern ThinkInk_290_Tricolor_Z94 display;

void init_eink();
void test_eink();

#endif