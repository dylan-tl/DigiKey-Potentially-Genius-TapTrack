/*
 * ADXL380.h - Arduino library for ADXL380 3-axis accelerometer
 *
 * This library provides functions to interface with the ADXL380 accelerometer
 * over SPI. It supports:
 * - X, Y, Z acceleration reading
 * - Configurable measurement ranges (±4g, ±8g, ±16g)
 * - Tap detection (single, double, triple)
 * - Multiple operating modes
 *
 * Based on Analog Devices ADXL38X driver
 * Modified for Arduino by Claude Code
 */

#ifndef ADXL380_H
#define ADXL380_H

#include <Arduino.h>
#include <SPI.h>

// ADXL380 Register Addresses
#define ADXL380_DEVID_AD        0x00
#define ADXL380_DEVID_MST       0x01
#define ADXL380_PART_ID         0x02
#define ADXL380_STATUS0         0x11
#define ADXL380_STATUS1         0x12
#define ADXL380_XDATA_H         0x15
#define ADXL380_XDATA_L         0x16
#define ADXL380_YDATA_H         0x17
#define ADXL380_YDATA_L         0x18
#define ADXL380_ZDATA_H         0x19
#define ADXL380_ZDATA_L         0x1A
#define ADXL380_OP_MODE         0x26
#define ADXL380_DIG_EN          0x27
#define ADXL380_REG_RESET       0x2A
#define ADXL380_TAP_THRESH      0x43
#define ADXL380_TAP_DUR         0x44
#define ADXL380_TAP_LATENT      0x45
#define ADXL380_TAP_WINDOW      0x46
#define ADXL380_TAP_CFG         0x47

// Reset Values
#define ADXL380_RESET_DEVID_AD  0xAD
#define ADXL380_RESET_DEVID_MST 0x1D
#define ADXL380_RESET_PART_ID   0x17

// Operating Modes
#define ADXL380_MODE_STDBY      0x00
#define ADXL380_MODE_HRT_SND    0x01
#define ADXL380_MODE_ULP        0x02
#define ADXL380_MODE_VLP        0x03
#define ADXL380_MODE_LP         0x04
#define ADXL380_MODE_RBW        0x08
#define ADXL380_MODE_HP         0x0C  // High Performance mode

// Range Settings
#define ADXL380_RANGE_4G        0x00
#define ADXL380_RANGE_8G        0x40
#define ADXL380_RANGE_16G       0x80

// Channel Enable (Enable X, Y, Z axes)
#define ADXL380_CH_EN_XYZ       0x70  // Bits 6:4 = 0b111 for XYZ

// Status1 Register Tap Detection Bits
#define ADXL380_SINGLE_TAP      0x01
#define ADXL380_DOUBLE_TAP      0x02
#define ADXL380_TRIPLE_TAP      0x04

// SPI Protocol
#define ADXL380_SPI_READ        0x01
#define ADXL380_SPI_WRITE       0x00

// Scale Factor for ±4g range: 133.3 µg/LSB = 0.0001333 g/LSB
#define ADXL380_SCALE_FACTOR_4G  0.0001333

// Function declarations

/*
 * Initialize the ADXL380 with specified CS pin
 * Must be called before any other functions
 *
 * @param csPin - The chip select pin to use for SPI communication
 * @param range - Initial measurement range (ADXL380_RANGE_4G, _8G, or _16G)
 * @return true if initialization successful, false otherwise
 */
bool adxl380_init(uint8_t csPin, uint8_t range = ADXL380_RANGE_4G);

/*
 * Perform a software reset of the ADXL380
 *
 * @return true if reset successful
 */
bool adxl380_soft_reset();

/*
 * Set the operating mode of the ADXL380
 * Preserves the range bits while changing mode
 *
 * @param mode - Operating mode (e.g., ADXL380_MODE_HP)
 */
void adxl380_set_op_mode(uint8_t mode);

/*
 * Set the measurement range
 * Updates both the hardware register and internal scale factor
 *
 * @param range - Range setting (ADXL380_RANGE_4G, _8G, or _16G)
 */
void adxl380_set_range(uint8_t range);

/*
 * Get the current measurement range in g-forces
 *
 * @return Current range (4.0, 8.0, or 16.0)
 */
float adxl380_get_current_range();

/*
 * Read raw 16-bit acceleration values for all three axes
 *
 * @param x - Pointer to store X-axis raw value
 * @param y - Pointer to store Y-axis raw value
 * @param z - Pointer to store Z-axis raw value
 */
void adxl380_get_raw_xyz(int16_t* x, int16_t* y, int16_t* z);

/*
 * Read acceleration values in g-forces for all three axes
 *
 * @param x - Pointer to store X-axis acceleration in g
 * @param y - Pointer to store Y-axis acceleration in g
 * @param z - Pointer to store Z-axis acceleration in g
 */
void adxl380_get_xyz_g(float* x, float* y, float* z);

/*
 * Read the tap detection status register
 *
 * @return Status byte containing tap flags
 */
uint8_t adxl380_get_tap_status();

/*
 * Check if a single tap was detected
 *
 * @return true if single tap detected
 */
bool adxl380_is_single_tap();

/*
 * Check if a double tap was detected
 *
 * @return true if double tap detected
 */
bool adxl380_is_double_tap();

/*
 * Check if a triple tap was detected
 *
 * @return true if triple tap detected
 */
bool adxl380_is_triple_tap();

/*
 * Configure tap detection with default or custom parameters
 *
 * @param tapThresh - Tap threshold (default: 0x20)
 * @param tapDur - Tap duration (default: 0x03)
 * @param tapLatent - Latency between taps (default: 0x10)
 * @param tapWindow - Window for detecting multiple taps (default: 0x20)
 */
void adxl380_setup_tap_detection(uint8_t tapThresh = 0x20,
                                  uint8_t tapDur = 0x03,
                                  uint8_t tapLatent = 0x10,
                                  uint8_t tapWindow = 0x20);

/*
 * Low-level register read/write functions
 * (Advanced users only - use higher-level functions when possible)
 */

/*
 * Read one or more bytes from ADXL380 registers
 *
 * @param regAddress - Starting register address
 * @param data - Buffer to store read data
 * @param len - Number of bytes to read
 */
void adxl380_read(uint8_t regAddress, uint8_t* data, uint8_t len);

/*
 * Write one or more bytes to ADXL380 registers
 *
 * @param regAddress - Starting register address
 * @param data - Data buffer to write
 * @param len - Number of bytes to write
 */
void adxl380_write(uint8_t regAddress, uint8_t* data, uint8_t len);

/*
 * Read a single register
 *
 * @param regAddress - Register address
 * @return Register value
 */
uint8_t adxl380_read_register(uint8_t regAddress);

/*
 * Write a single register
 *
 * @param regAddress - Register address
 * @param value - Value to write
 */
void adxl380_write_register(uint8_t regAddress, uint8_t value);

#endif // ADXL380_H
