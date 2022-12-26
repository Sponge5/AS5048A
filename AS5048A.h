#ifndef __AS5048A_H__
#define __AS5048A_H__

#define PI 3.14

#define AS5048A_NOP                         0x0000
#define AS5048A_CLEAR_ERROR_FLAG            0x0001
#define AS5048A_PROGRAMMING_CONTROL         0x0003
#define AS5048A_OTP_REGISTER_ZERO_POS_HIGH  0x0016
#define AS5048A_OTP_REGISTER_ZERO_POS_LOW   0x0017
#define AS5048A_DIAG_AGC                    0x3FFD
#define AS5048A_MAGNITUDE                   0x3FFE
#define AS5048A_ANGLE                       0x3FFF

#define AS5048A_AGC_FLAG                    0xFF
#define AS5048A_ERROR_PARITY_FLAG           0x04
#define AS5048A_ERROR_COMMAND_INVALID_FLAG  0x02
#define AS5048A_ERROR_FRAMING_FLAG          0x01

#define AS5048A_DIAG_COMP_HIGH              0x2000
#define AS5048A_DIAG_COMP_LOW               0x1000
#define AS5048A_DIAG_COF                    0x0800
#define AS5048A_DIAG_OCF                    0x0400

#define AS5048A_MAX_VALUE                   8191.0

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

typedef struct
{
    spi_device_handle_t spi;
	uint8_t _cs;
	bool errorFlag;
	bool ocfFlag; // Avoid printing OCF flag everytime
	uint16_t position;
	uint8_t esp32_delay;
}
AS5048A_t;

AS5048A_t AS5048A;

uint8_t AS5048A_spiCalcEvenParity(uint16_t);

/**
 * Closes the SPI connection
 */
void AS5048A_close();

/*
 * Read a register from the sensor
 * Takes the address of the register as a 16 bit uint16_t
 * Returns the value of the register
 */
uint16_t AS5048A_read(uint16_t registerAddress);

/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the 16 bit uint16_t of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t AS5048A_write(uint16_t registerAddress, uint16_t data);

/**
 *	Constructor
 */
void AS5048A_init(spi_host_device_t host, gpio_num_t cs);

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int16_t} between -2^13 and 2^13
 */
int16_t AS5048A_getRotation();

/**
 * Returns the raw angle directly from the sensor
 */
int16_t AS5048A_getRawRotation();

/**
 * Get the rotation of the sensor relative to the zero position in degrees.
 *
 * @return {double} between 0 and 360
 */
double AS5048A_getRotationInDegrees();

/**
 * Get the rotation of the sensor relative to the zero position in radians.
 *
 * @return {double} between 0 and 2 * PI
 */
double AS5048A_getRotationInRadians();

/**
 * returns the value of the state register
 * @return 16 bit uint16_t containing flags
 */
uint16_t AS5048A_getState();

/**
 * Print the diagnostic register of the sensor
 */
void AS5048A_printState();

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5048A_getGain();

/*
 * Get and clear the error register by reading it
 */
uint16_t AS5048A_getErrors();

/**
 * Get diagnostic
 */
uint16_t AS5048A_getDiagnostic();

/*
 * Set the zero position
 */
void AS5048A_setZeroPosition(uint16_t arg_position);

/*
 * Returns the current zero position
 */
uint16_t AS5048A_getZeroPosition();

/*
 * Check if an error has been encountered.
 */
bool AS5048A_error();

void AS5048A_nop();

#endif // __AS5048A_H__
