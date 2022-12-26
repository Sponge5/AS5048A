#include "AS5048A.h"

//#define AS5048A_DEBUG

static const char * TAG = "AS5048A";

/**
 * Constructor
 */
void AS5048A_init(spi_host_device_t host, gpio_num_t cs)
{
    esp_err_t ret;
    gpio_set_direction(cs, GPIO_MODE_OUTPUT);
    gpio_set_level(cs, 1);
    spi_device_interface_config_t devcfg = {
        .mode = 1,
        //.clock_speed_hz = SPI_MASTER_FREQ_8M/2,
        .clock_speed_hz = SPI_MASTER_FREQ_10M,
        //.input_delay_ns = 20,
        .spics_io_num = cs,
        //.flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 4,
    };
    ESP_LOGD(TAG, "Adding encoder device to SPI bus");
    //Attach device to the SPI bus
    ret = spi_bus_add_device(host, &devcfg, &(AS5048A.spi));
    if(ret != ESP_OK)
        ESP_LOGE(TAG, "Adding encoder device to SPI bus failed");
    ESP_ERROR_CHECK(ret);
    AS5048A.errorFlag = false;
    AS5048A.ocfFlag = false;
    AS5048A.position = 0;
    AS5048A.esp32_delay = 50;
}

/**
 * Utility function used to calculate even parity of an unigned 16 bit integer
 */
uint8_t AS5048A_spiCalcEvenParity(uint16_t value)
{
	uint8_t cnt = 0;

	for (uint8_t i = 0; i < 16; i++)
	{
		if (value & 0x1)
		{
			cnt++;
		}
		value >>= 1;
	}
	return cnt & 0x1;
}

/**
 * Get the rotation of the sensor relative to the zero position.
 *
 * @return {int16_t} between -2^13 and 2^13
 */
int16_t AS5048A_getRotation()
{
	uint16_t data;
	int16_t rotation;

	data = AS5048A_getRawRotation();
	rotation = (int16_t)data - (int16_t)(AS5048A.position);
	if (rotation > AS5048A_MAX_VALUE)
		rotation = -((0x3FFF) - rotation); //more than -180

	return rotation;
}

/**
 * Returns the raw angle directly from the sensor
 */
int16_t AS5048A_getRawRotation()
{
	return AS5048A_read(AS5048A_ANGLE);
}

/**
  * Get the rotation of the sensor relative to the zero position in degrees.
  *
  * @return {double} between 0 and 360
  */

double AS5048A_getRotationInDegrees()
{
	int16_t rotation = AS5048A_getRotation();
	double degrees = 360.0 * (rotation + AS5048A_MAX_VALUE) / (AS5048A_MAX_VALUE * 2.0);
	return degrees;
}

/**
  * Get the rotation of the sensor relative to the zero position in radians.
  *
  * @return {double} between 0 and 2 * PI
  */

double AS5048A_getRotationInRadians()
{
	int16_t rotation = AS5048A_getRotation();
	double radians = PI * (rotation + AS5048A_MAX_VALUE) / AS5048A_MAX_VALUE;
	return radians;
}

/**
 * returns the value of the state register
 * @return unsigned 16 bit integer containing flags
 */
uint16_t AS5048A_getState()
{
	return AS5048A_read(AS5048A_DIAG_AGC);
}

/**
 * Returns the value used for Automatic Gain Control (Part of diagnostic
 * register)
 */
uint8_t AS5048A_getGain()
{
	uint16_t data = AS5048A_getState();
	return (uint8_t)(data & AS5048A_AGC_FLAG);
}

/**
 * Get diagnostic
 */
uint16_t AS5048A_getDiagnostic()
{
	uint16_t data = AS5048A_getState();
	if (data & AS5048A_DIAG_COMP_HIGH)
    {
		ESP_LOGD(TAG, "COMP high");
    }
    else if (data & AS5048A_DIAG_COMP_LOW)
    {
		ESP_LOGD(TAG, "COMP low");
    }
    else if (data & AS5048A_DIAG_COF)
    {
		ESP_LOGD(TAG, "CORDIC overflow");
    }
    else if (data & AS5048A_DIAG_OCF && AS5048A.ocfFlag == false)
	{
		AS5048A.ocfFlag = true;
		ESP_LOGD(TAG, "Offset compensation finished");
	}
    return data;
}

/*
 * Get and clear the error register by reading it
 */
uint16_t AS5048A_getErrors()
{
	uint16_t error = AS5048A_read(AS5048A_CLEAR_ERROR_FLAG);
	if (error & AS5048A_ERROR_PARITY_FLAG)
    {
		ESP_LOGD(TAG, "Parity Error");
    }
    else if (error & AS5048A_ERROR_COMMAND_INVALID_FLAG)
    {
		ESP_LOGD(TAG, "Command invalid");
    }
    else if (error & AS5048A_ERROR_FRAMING_FLAG)
    {
		ESP_LOGD(TAG, "Framing error");
    }
    else
    {
		ESP_LOGD(TAG, "Different error %u", error);
    }
    return error;
}

/*
 * Set the zero position
 */
void AS5048A_setZeroPosition(uint16_t position)
{
	AS5048A.position = position % 0x3FFF;
}

/*
 * Returns the current zero position
 */
uint16_t AS5048A_getZeroPosition()
{
	return AS5048A.position;
}

/*
 * Check if an error has been encountered.
 */
bool AS5048A_error()
{
	return AS5048A.errorFlag;
}

void AS5048A_nop()
{
	AS5048A_read(AS5048A_NOP);
}

/*
 * Read a register from the sensor
 * Takes the address of the register as an unsigned 16 bit
 * Returns the value of the register
 */
uint16_t AS5048A_read(uint16_t registerAddress)
{
    esp_err_t err;
	uint16_t command = 0x4000 | registerAddress;
    uint32_t dataToSend = 0,
             response = 0;

	//Add a parity bit on the the MSB
	command |= (uint16_t)(AS5048A_spiCalcEvenParity(command) << 0xF);

    dataToSend |= command;

    ESP_LOGD(TAG, "Register 0x%X command 0x%X", registerAddress, command);

    dataToSend |= SPI_SWAP_DATA_TX(*(uint32_t *)&dataToSend, 32);

 	//SPI - begin transaction
    spi_transaction_t t =
    {
         .flags = 0,
         .length = 32,
         .user = NULL,
         .tx_buffer = &dataToSend,
         .rx_buffer = &response
     };

    err = spi_device_polling_transmit(AS5048A.spi, &t);
    if(err != ESP_OK) return 0;

    response &= 0xFFFF;
    response = SPI_SWAP_DATA_RX(*(uint16_t *)&response, 16);

    ESP_LOGD(TAG, "Response 0x%X", response);

	//Check if the error bit is set
	if(response & 0x4000)
	{
	    ESP_LOGD(TAG, "Setting error bit");
		AS5048A.errorFlag = true;
	}
	else
		AS5048A.errorFlag = false;

	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}

/*
 * Write to a register
 * Takes the 16-bit  address of the target register and the unsigned 16 bit of data
 * to be written to that register
 * Returns the value of the register after the write has been performed. This
 * is read back from the sensor to ensure a sucessful write.
 */
uint16_t AS5048A_write(uint16_t registerAddress, uint16_t data)
{
    esp_err_t err;
	uint16_t command = registerAddress;
    uint32_t response = 0;

	//Add a parity bit on the the MSB
	command |= (uint16_t)(AS5048A_spiCalcEvenParity(command) << 0xF);

	uint32_t dataToSend = (uint32_t)command << 16 | (uint32_t)data;

    ESP_LOGD(TAG, "write(): reg 0x%X dataToSend 0x%X", registerAddress, dataToSend);

    err = spi_device_acquire_bus(AS5048A.spi, portMAX_DELAY);
    if(err != ESP_OK) return 0;

	//Craft another packet including the data and parity
	dataToSend |= (uint16_t)(AS5048A_spiCalcEvenParity(dataToSend) << 0xF);

    dataToSend = SPI_SWAP_DATA_TX(*(uint32_t *)&dataToSend, 32);

 	//SPI - begin transaction
    spi_transaction_t t = {
         .flags = 0,
         .length = 32,
         .user = NULL,
         .tx_buffer = &dataToSend,
         .rx_buffer = &response
    };

    err = spi_device_polling_transmit(AS5048A.spi, &t);
    if(err != ESP_OK) return 0;

    response = SPI_SWAP_DATA_RX(*(uint32_t *)&response, 32);

    ESP_LOGD(TAG, "Response 0x%X", response);

	//Return the data, stripping the parity and error bits
	return response & ~0xC000;
}

