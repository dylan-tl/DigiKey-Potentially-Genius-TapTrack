/*
 * ADXL380 Accelerometer Application
 *
 * This sketch demonstrates the ADXL380 3-axis accelerometer library
 * with drop detection and e-ink display visualization.
 *
 * Hardware Connections:
 * ADXL380:
 * - CS:   Pin 11
 * - MOSI: Pin 11 (SPI)
 * - MISO: Pin 12 (SPI)
 * - SCK:  Pin 13 (SPI)
 *
 * E-Ink Display:
 * - See eink.h for display connections
 *
 * Based on Analog Devices ADXL38X driver
 * Modified for Arduino by Claude Code
 */

#include <SPI.h>
#include "ADXL380.h"
#include "eink.h"


/* ------------------ SETTINGS HERE ------------------ */
#define DROP_THRESH 10    // The dropped threshold in gs
#define TRACKING_TIME 2880 // The time period for which to record drops in minutes
#define AVERAGE_NUM 4     // The number of samples to average
#define SAMPLES_PER_SECOND 200 // The number of samples per second


// Pin Configuration
#define ADXL380_CS_PIN 11

// Application variables
unsigned int count = 0;
int display_line = 0;
bool dropped = false;
bool displayed = false;

float max_drop = 0;
float all_time_max = 0;

bool started = false; // changed for testing

int average_counter = 0;

float x_sum = 0;
float y_sum = 0;
float z_sum = 0;
float max_drop_average = 0;

int count_number = 0;


// Helper function to find maximum g-force across axes
float max_g(float x, float y, float z, float current_max);

/*
 * Arduino Setup
 */
void setup() {
  Serial.begin(115200);
  delay(1000);
  /*
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  */
  Serial.println("ADXL380 Accelerometer Test");
  Serial.println("===========================");

  // Initialize SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // Initialize ADXL380 with CS pin and ±16g range
  if (!adxl380_init(ADXL380_CS_PIN, ADXL380_RANGE_16G)) {
    Serial.println("ADXL380 initialization failed!");
    while (1) {
      delay(1000);
    }
  }

  // Setup tap detection
  adxl380_setup_tap_detection();

  Serial.println("\nStarting acceleration measurements...\n");

  // Initialize e-ink display
  init_eink();
  //test_eink();i
  /*
    display.clearBuffer();
  display.setTextSize(3);
  display.setCursor((display.width() - 130) / 4, (display.height() - 24) / 2);
  display.setTextColor(EPD_BLACK);
  display.print("Tap to start");
  display.display();
  display.clearBuffer();
  */
  // 296 lines on the display
  // time per line in min = total tracking time in minutes / 296
  // time per line in s = (total tracking time in minutes / 296) * 60
  // time per line in ms = (total tracking time in minutes / 296) * 60 * 1000
  // counts per second = 200
  // counts per line = (total tracking time in minutes / 296) * 60 * 200


  display.clearBuffer();

  count_number = (float)TRACKING_TIME / 296.0 * 60 * SAMPLES_PER_SECOND;
  Serial.println("THE COUNT NUMBER IS AS FOLLOWS");
  Serial.println(count_number);
}

/*
 * Arduino Main Loop
 */
void loop() {
  float x, y, z;

  // Read acceleration in g-forces
  adxl380_get_xyz_g(&x, &y, &z);
/*
  // Print acceleration values
  Serial.print("X: ");
  Serial.print(x, 4);
  Serial.print(" g\t");

  Serial.print("Y: ");
  Serial.print(y, 4);
  Serial.print(" g\t");

  Serial.print("Z: ");
  Serial.print(z, 4);
  Serial.println(" g");
 */ 

  // Accumulate x, y, z values for averaging
  if(average_counter < AVERAGE_NUM && started){
    x_sum += x;
    y_sum += y;
    z_sum += z;
    average_counter++;
  }
  
  // When we have enough samples, calculate average and find max
  if(average_counter >= AVERAGE_NUM){
    float x_avg = x_sum / AVERAGE_NUM;
    float y_avg = y_sum / AVERAGE_NUM;
    float z_avg = z_sum / AVERAGE_NUM;
    
    // Find max of the averaged values
    max_drop_average = max_g(x_avg, y_avg, z_avg, 0.0);
    
    // Track the maximum of these averaged values
    if(max_drop_average > max_drop){
      max_drop = max_drop_average;
    }
    
    // Reset for next averaging window
    x_sum = 0;
    y_sum = 0;
    z_sum = 0;
    average_counter = 0;
  }

  Serial.print(max_drop_average);
  Serial.print(",");
  Serial.println(max_drop);



  delay(5);  // Sample at ~100 Hz

  if(started){
  count++;
  }

  if(!started && adxl380_is_triple_tap()){
    started = true;
    max_drop = 0;
        display.clearBuffer();
  display.setTextSize(3);
  display.setCursor((display.width() - 130) / 4, (display.height() - 24) / 2);
  display.setTextColor(EPD_BLACK);
  display.print("Started!");
  display.display();
  display.clearBuffer();
    delay(1000);
  }


  // Update display every 1000 samples (~10 seconds)
  if(count > count_number){
    // If g force is above 8, make the line red to show significant Gs
    if(max_drop > DROP_THRESH){
      display.drawLine(display_line+5, display.height()/2-int(max_drop*2) + 20,
                      display_line+5, display.height()/2+int(max_drop*2) + 20, EPD_RED);
    } else {
      display.drawLine(display_line+5, display.height()/2-int(max_drop*2) + 20,
                      display_line+5, display.height()/2+int(max_drop*2) + 20, EPD_BLACK);
    }

    if(max_drop > all_time_max){
      all_time_max = max_drop;
    }

    Serial.print("ALL TIME MAX: ");
    Serial.println(all_time_max);

    count = 0;
    max_drop = 0;

    // Draw a white rectange to clear the text from last refresh
    display.fillRect(0, 0, display.width(), display.height()/3, EPD_WHITE);
    // Set the text size and cursor location for the g force text
    display.setTextSize(2);
    display.setCursor(170, 15);

    if(all_time_max > DROP_THRESH){
      // If we detect a drop, print everything in red
      display.setTextColor(EPD_RED);
      // Print the g force value
      display.print(String(all_time_max, 1)+"g");
      // Set up and print the dropped text
      display.setCursor(25, 10);
      display.setTextSize(3);
      display.print("DROPPED");
    } else {
      // If there was no drop, print everything in black
      display.setTextColor(EPD_BLACK);
      // Print the g force value
      display.print(String(all_time_max, 1)+"g");
      // Set up and print the safe text
      display.setCursor(25, 10);
      display.setTextSize(3);
      display.print("SAFE");
    }

    // Display the changes
    display.display();
    // Increment the display line so that the next graph line is drawn to the right of the last one
    display_line++;

    // If we've drawn all the graph lines, end the program
    if(display_line >= 291) {
      while(true){delay(10);}
    }
    
  }

Serial.println(count);
  
}

float max_g(float x, float y, float z, float current_max){
  /*
   * Find maximum absolute g-force across all three axes
   * Compares with current maximum and returns the highest value
   */
  float max_g = current_max;
  x = abs(x);
  y = abs(y);
  z = abs(z);

  if (x > current_max) {
    current_max = x;
  }
  if (y > current_max) {
    current_max = y;
  }
  if (z > current_max) {
    current_max = z;
  }

  return current_max;
}
  