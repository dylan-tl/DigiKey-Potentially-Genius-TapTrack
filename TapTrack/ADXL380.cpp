/*
 * ADXL380.cpp - Arduino library for ADXL380 3-axis accelerometer
 *
 * Implementation file for ADXL380 library
 *
 * Based on Analog Devices ADXL38X driver
 * Modified for Arduino by Claude Code
 */

#include "ADXL380.h"

// Static variables to store configuration
static uint8_t _csPin = 0;
static float _currentRange = 4.0;
static float _scaleFactor = ADXL380_SCALE_FACTOR_4G;

/*
 * SPI Read Function
 * Reads one or more bytes from the ADXL380
 */
void adxl380_read(uint8_t regAddress, uint8_t* data, uint8_t len) {
  digitalWrite(_csPin, LOW);

  // Send address with read bit
  SPI.transfer((regAddress << 1) | ADXL380_SPI_READ);

  // Read data bytes
  for (uint8_t i = 0; i < len; i++) {
    data[i] = SPI.transfer(0x00);
  }

  digitalWrite(_csPin, HIGH);
}

/*
 * SPI Write Function
 * Writes one or more bytes to the ADXL380
 */
void adxl380_write(uint8_t regAddress, uint8_t* data, uint8_t len) {
  digitalWrite(_csPin, LOW);

  // Send address with write bit
  SPI.transfer((regAddress << 1) | ADXL380_SPI_WRITE);

  // Write data bytes
  for (uint8_t i = 0; i < len; i++) {
    SPI.transfer(data[i]);
  }

  digitalWrite(_csPin, HIGH);
}

/*
 * Write a single register
 */
void adxl380_write_register(uint8_t regAddress, uint8_t value) {
  adxl380_write(regAddress, &value, 1);
}

/*
 * Read a single register
 */
uint8_t adxl380_read_register(uint8_t regAddress) {
  uint8_t value;
  adxl380_read(regAddress, &value, 1);
  return value;
}

/*
 * Software Reset
 */
bool adxl380_soft_reset() {
  uint8_t resetCode = 0x52;
  adxl380_write(ADXL380_REG_RESET, &resetCode, 1);
  delay(10);  // Wait for reset to complete
  return true;
}

/*
 * Set Operating Mode (with standby transition for safety)
 * Preserves the range bits [7:6] while setting mode bits [3:0]
 */
void adxl380_set_op_mode(uint8_t mode) {
  // Read current OP_MODE to preserve range bits
  uint8_t opMode = adxl380_read_register(ADXL380_OP_MODE);
  uint8_t rangeBits = opMode & 0xC0;  // Preserve bits 7:6 (range)

  // First transition to standby for safe mode change
  adxl380_write_register(ADXL380_OP_MODE, rangeBits | ADXL380_MODE_STDBY);
  delay(1);

  // Then set the desired mode while preserving range
  adxl380_write_register(ADXL380_OP_MODE, rangeBits | mode);
  delay(1);
}

/*
 * Set Measurement Range
 * Updates both the register and the scale factor
 */
void adxl380_set_range(uint8_t range) {
  uint8_t opMode = adxl380_read_register(ADXL380_OP_MODE);

  // Clear range bits and set new range
  opMode = (opMode & 0x3F) | range;

  // Transition to standby first
  adxl380_write_register(ADXL380_OP_MODE, ADXL380_MODE_STDBY);
  delay(1);

  // Write new mode with range
  adxl380_write_register(ADXL380_OP_MODE, opMode);
  delay(1);

  // Update scale factor based on range
  switch(range) {
    case ADXL380_RANGE_4G:
      _currentRange = 4.0;
      _scaleFactor = ADXL380_SCALE_FACTOR_4G;
      break;
    case ADXL380_RANGE_8G:
      _currentRange = 8.0;
      _scaleFactor = ADXL380_SCALE_FACTOR_4G * 2.0;
      break;
    case ADXL380_RANGE_16G:
      _currentRange = 16.0;
      _scaleFactor = ADXL380_SCALE_FACTOR_4G * 4.0;
      break;
  }
}

/*
 * Get the current measurement range
 */
float adxl380_get_current_range() {
  return _currentRange;
}

/*
 * Initialize ADXL380
 */
bool adxl380_init(uint8_t csPin, uint8_t range) {
  // Store CS pin
  _csPin = csPin;

  // Configure CS pin as output
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  // Verify device ID
  uint8_t devIdAd = adxl380_read_register(ADXL380_DEVID_AD);
  uint8_t devIdMst = adxl380_read_register(ADXL380_DEVID_MST);
  uint8_t partId = adxl380_read_register(ADXL380_PART_ID);

  Serial.print("Device ID AD: 0x");
  Serial.println(devIdAd, HEX);
  Serial.print("Device ID MST: 0x");
  Serial.println(devIdMst, HEX);
  Serial.print("Part ID: 0x");
  Serial.println(partId, HEX);

  // Check if device IDs match expected values
  if (devIdAd != ADXL380_RESET_DEVID_AD ||
      devIdMst != ADXL380_RESET_DEVID_MST ||
      partId != ADXL380_RESET_PART_ID) {
    Serial.println("ERROR: Device ID mismatch!");
    return false;
  }

  // Perform soft reset
  Serial.println("Performing soft reset...");
  adxl380_soft_reset();

  // Enable X, Y, Z channels
  Serial.println("Enabling XYZ channels...");
  adxl380_write_register(ADXL380_DIG_EN, ADXL380_CH_EN_XYZ);

  // Set measurement range
  Serial.print("Setting range to ±");
  if (range == ADXL380_RANGE_4G) {
    Serial.print("4");
  } else if (range == ADXL380_RANGE_8G) {
    Serial.print("8");
  } else if (range == ADXL380_RANGE_16G) {
    Serial.print("16");
  }
  Serial.println("g...");
  adxl380_set_range(range);

  // Set to High Performance mode
  Serial.println("Setting High Performance mode...");
  adxl380_set_op_mode(ADXL380_MODE_HP);

  delay(100);  // Allow mode to settle

  Serial.println("ADXL380 initialization complete!");
  return true;
}

/*
 * Read raw acceleration data for X, Y, Z axes
 * Returns 16-bit signed values
 */
void adxl380_get_raw_xyz(int16_t* x, int16_t* y, int16_t* z) {
  uint8_t data[6];

  // Read all 6 bytes (X, Y, Z data registers)
  adxl380_read(ADXL380_XDATA_H, data, 6);

  // Combine high and low bytes (big-endian format)
  *x = (int16_t)((data[0] << 8) | data[1]);
  *y = (int16_t)((data[2] << 8) | data[3]);
  *z = (int16_t)((data[4] << 8) | data[5]);
}

/*
 * Read acceleration data in g-forces
 */
void adxl380_get_xyz_g(float* x, float* y, float* z) {
  int16_t rawX, rawY, rawZ;

  adxl380_get_raw_xyz(&rawX, &rawY, &rawZ);

  // Convert to g-forces using scale factor
  *x = rawX * _scaleFactor;
  *y = rawY * _scaleFactor;
  *z = rawZ * _scaleFactor;
}

/*
 * Read tap detection status
 * Returns status byte with tap flags
 */
uint8_t adxl380_get_tap_status() {
  return adxl380_read_register(ADXL380_STATUS1);
}

/*
 * Check if single tap was detected
 */
bool adxl380_is_single_tap() {
  uint8_t status = adxl380_get_tap_status();
  return (status & ADXL380_SINGLE_TAP) != 0;
}

/*
 * Check if double tap was detected
 */
bool adxl380_is_double_tap() {
  uint8_t status = adxl380_get_tap_status();
  return (status & ADXL380_DOUBLE_TAP) != 0;
}

/*
 * Check if triple tap was detected
 */
bool adxl380_is_triple_tap() {
  uint8_t status = adxl380_get_tap_status();
  return (status & ADXL380_TRIPLE_TAP) != 0;
}

/*
 * Setup tap detection (basic configuration for future development)
 * This is a placeholder - tap detection parameters can be tuned
 * based on application requirements
 */
void adxl380_setup_tap_detection(uint8_t tapThresh,
                                  uint8_t tapDur,
                                  uint8_t tapLatent,
                                  uint8_t tapWindow) {
  // Set tap threshold
  adxl380_write_register(ADXL380_TAP_THRESH, tapThresh);

  // Set tap duration
  adxl380_write_register(ADXL380_TAP_DUR, tapDur);

  // Set latent time between taps
  adxl380_write_register(ADXL380_TAP_LATENT, tapLatent);

  // Set window for detecting second tap
  adxl380_write_register(ADXL380_TAP_WINDOW, tapWindow);

  // Enable tap detection on all axes
  uint8_t tapCfg = 0x07;  // Enable X, Y, Z for tap detection
  adxl380_write_register(ADXL380_TAP_CFG, tapCfg);

  Serial.println("Tap detection configured");
}
