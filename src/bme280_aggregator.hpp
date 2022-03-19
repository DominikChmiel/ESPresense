#pragma once

#include "Arduino.h"

#include <SPI.h>
#include <Wire.h>

/*
	Adapted from Adafruit_BME280.h + .cpp

	https://www.bosch-sensortec.com/media/boschsensortec/downloads/environmental_sensors_2/humidity_sensors_1/bme280/bst-bme280-ds002.pdf

	Raw data:
	Pressure + temperature 20 bit, stored in 32 bit signed int
	Humidity: 16 bit, 32 bit signed int

	Sensor ranges:
	Temperature: -40..85*C, resolution 0.01 C => 12500 => 14 bit
	Pressure: 300-1100 hPa
	Humidity: 0-100% => 0-128 => 7 bit
 */

using sensor_data = struct sensor_data_s {
	int32_t temperature;   // : 20;
	int32_t pressure;      // : 20;
	int32_t humidity;      // : 16;

	auto getTemp() -> int32_t {
		// Returns temperature in .001 DegC steps
		return (temperature * 10) >> 8;
	};
	auto getPress() -> int32_t {
		// FIXME: Might need widening here
		return (pressure * 25) >> 6;
	};
	auto getHum() -> int32_t {
		// Returns humidity from 0 to 10000 (where 10k is 100%)
		return (humidity * 100) >> 10;
	};

	auto getTempStr() -> String {
		char full_str[128];
		snprintf(full_str,
				 sizeof(full_str),
				 "%d.%03d",
				 getTemp() / 1000,
				 getTemp() % 1000 + (1000 * (getTemp() < 0))   // Correct for negative % returning remainder (=> negative for x < 0)
		);
		return String(full_str);
	}

	auto getPressStr() -> String {
		char full_str[128];
		snprintf(full_str,
				 sizeof(full_str),
				 "%d.%02d",
				 getPress() / 100,
				 getPress() % 100
		);
		return String(full_str);
	}

	auto getHumStr() -> String {
		char full_str[128];
		snprintf(full_str,
				 sizeof(full_str),
				 "%d.%04d",
				 getHum() / 100,
				 getHum() % 100
		);
		return String(full_str);
	}

	auto toString() -> String {
		char full_str[128];
		snprintf(full_str,
				 sizeof(full_str),
				 "temperature=%s,pressure=%s,humidity=%s",
				 getTempStr().c_str(),
				 getPressStr().c_str(),
				 getHumStr().c_str());
		return String(full_str);
	}
};

/*!
 *  @brief  default I2C address
 */
#define BME280_ADDRESS (0x76)   // Primary I2C Address

/*!
 *  @brief Register addresses
 */
enum {
	BME280_REGISTER_DIG_T1 = 0x88,
	BME280_REGISTER_DIG_T2 = 0x8A,
	BME280_REGISTER_DIG_T3 = 0x8C,

	BME280_REGISTER_DIG_P1 = 0x8E,
	BME280_REGISTER_DIG_P2 = 0x90,
	BME280_REGISTER_DIG_P3 = 0x92,
	BME280_REGISTER_DIG_P4 = 0x94,
	BME280_REGISTER_DIG_P5 = 0x96,
	BME280_REGISTER_DIG_P6 = 0x98,
	BME280_REGISTER_DIG_P7 = 0x9A,
	BME280_REGISTER_DIG_P8 = 0x9C,
	BME280_REGISTER_DIG_P9 = 0x9E,

	BME280_REGISTER_DIG_H1 = 0xA1,
	BME280_REGISTER_DIG_H2 = 0xE1,
	BME280_REGISTER_DIG_H3 = 0xE3,
	BME280_REGISTER_DIG_H4 = 0xE4,
	BME280_REGISTER_DIG_H5 = 0xE5,
	BME280_REGISTER_DIG_H6 = 0xE7,

	BME280_REGISTER_CHIPID    = 0xD0,
	BME280_REGISTER_VERSION   = 0xD1,
	BME280_REGISTER_SOFTRESET = 0xE0,

	BME280_REGISTER_CAL26 = 0xE1,   // R calibration stored in 0xE1-0xF0

	BME280_REGISTER_CONTROLHUMID = 0xF2,
	BME280_REGISTER_STATUS       = 0XF3,
	BME280_REGISTER_CONTROL      = 0xF4,
	BME280_REGISTER_CONFIG       = 0xF5,
	BME280_REGISTER_PRESSUREDATA = 0xF7,
	BME280_REGISTER_TEMPDATA     = 0xFA,
	BME280_REGISTER_HUMIDDATA    = 0xFD
};

/**************************************************************************/
/*!
	@brief  calibration data
*/
/**************************************************************************/
typedef struct {
	uint16_t dig_T1;   ///< temperature compensation value
	int16_t  dig_T2;   ///< temperature compensation value
	int16_t  dig_T3;   ///< temperature compensation value

	uint16_t dig_P1;   ///< pressure compensation value
	int16_t  dig_P2;   ///< pressure compensation value
	int16_t  dig_P3;   ///< pressure compensation value
	int16_t  dig_P4;   ///< pressure compensation value
	int16_t  dig_P5;   ///< pressure compensation value
	int16_t  dig_P6;   ///< pressure compensation value
	int16_t  dig_P7;   ///< pressure compensation value
	int16_t  dig_P8;   ///< pressure compensation value
	int16_t  dig_P9;   ///< pressure compensation value

	uint8_t dig_H1;   ///< humidity compensation value
	int16_t dig_H2;   ///< humidity compensation value
	uint8_t dig_H3;   ///< humidity compensation value
	int16_t dig_H4;   ///< humidity compensation value
	int16_t dig_H5;   ///< humidity compensation value
	int8_t  dig_H6;   ///< humidity compensation value
} bme280_calib_data;
/*=========================================================================*/

/**************************************************************************/
/*!
	@brief  Class that stores state and functions for interacting with BME280 IC
*/
/**************************************************************************/
class BME280Aggregator {
public:
	/**************************************************************************/
	/*!
		@brief  sampling rates
	*/
	/**************************************************************************/
	enum sensor_sampling {
		SAMPLING_NONE = 0b000,
		SAMPLING_X1   = 0b001,
		SAMPLING_X2   = 0b010,
		SAMPLING_X4   = 0b011,
		SAMPLING_X8   = 0b100,
		SAMPLING_X16  = 0b101
	};

	/**************************************************************************/
	/*!
		@brief  power modes
	*/
	/**************************************************************************/
	enum sensor_mode { MODE_SLEEP = 0b00, MODE_FORCED = 0b01, MODE_NORMAL = 0b11 };

	/**************************************************************************/
	/*!
		@brief  filter values
	*/
	/**************************************************************************/
	enum sensor_filter { FILTER_OFF = 0b000, FILTER_X2 = 0b001, FILTER_X4 = 0b010, FILTER_X8 = 0b011, FILTER_X16 = 0b100 };

	/**************************************************************************/
	/*!
		@brief  standby duration in ms
	*/
	/**************************************************************************/
	enum standby_duration {
		STANDBY_MS_0_5  = 0b000,
		STANDBY_MS_10   = 0b110,
		STANDBY_MS_20   = 0b111,
		STANDBY_MS_62_5 = 0b001,
		STANDBY_MS_125  = 0b010,
		STANDBY_MS_250  = 0b011,
		STANDBY_MS_500  = 0b100,
		STANDBY_MS_1000 = 0b101
	};

	auto begin(uint8_t addr = BME280_ADDRESS, TwoWire *theWire = &Wire) -> bool;
	auto init() -> bool;

	void setSampling(sensor_mode      mode          = MODE_NORMAL,
					 sensor_sampling  tempSampling  = SAMPLING_X16,
					 sensor_sampling  pressSampling = SAMPLING_X16,
					 sensor_sampling  humSampling   = SAMPLING_X16,
					 sensor_filter    filter        = FILTER_OFF,
					 standby_duration duration      = STANDBY_MS_0_5);

	auto readAllSensors() -> sensor_data;

	auto sensorID() -> uint32_t;

protected:
	TwoWire * m_wire;   //!< pointer to a TwoWire object
	SPIClass *m_spi;    //!< pointer to SPI object

	void readCoefficients();
	auto isReadingCalibration() -> bool;

	auto adaptTemp(uint32_t raw_temp) -> int32_t;
	auto adaptPressure(uint32_t raw_press) -> int32_t;
	auto adaptHumidity(uint32_t raw_humidity) -> int32_t;

	void write8(byte reg, byte value);
	auto read8(byte reg) -> uint8_t;
	auto read16(byte reg) -> uint16_t;
	auto readS16(byte reg) -> int16_t;
	auto read16_LE(byte reg) -> uint16_t;   // little endian
	auto readS16_LE(byte reg) -> int16_t;   // little endian

	uint8_t m_i2caddr;    //!< I2C addr for the TwoWire interface
	int32_t m_sensorId;   //!< ID of the BME Sensor
	int32_t m_tFine;      //!< temperature with high resolution, stored as an attribute
						  //!< as this is used for temperature compensation reading
						  //!< humidity and pressure

	bme280_calib_data m_bme280Calib;   //!< here calibration data is stored

	/**************************************************************************/
	/*!
		@brief  config register
	*/
	/**************************************************************************/
	struct config_s {
		// inactive duration (standby time) in normal mode
		// 000 = 0.5 ms
		// 001 = 62.5 ms
		// 010 = 125 ms
		// 011 = 250 ms
		// 100 = 500 ms
		// 101 = 1000 ms
		// 110 = 10 ms
		// 111 = 20 ms
		unsigned int t_sb : 3;   ///< inactive duration (standby time) in normal mode

		// filter settings
		// 000 = filter off
		// 001 = 2x filter
		// 010 = 4x filter
		// 011 = 8x filter
		// 100 and above = 16x filter
		unsigned int filter : 3;   ///< filter settings

		// unused - don't set
		unsigned int none : 1;       ///< unused - don't set
		unsigned int spi3w_en : 1;   ///< unused - don't set

		/// @return combined config register
		auto get() -> unsigned int {
			return (t_sb << 5) | (filter << 2) | spi3w_en;
		}
	};
	config_s m_configReg;   //!< config register object

	/**************************************************************************/
	/*!
		@brief  ctrl_meas register
	*/
	/**************************************************************************/
	struct ctrl_meas_s {
		// temperature oversampling
		// 000 = skipped
		// 001 = x1
		// 010 = x2
		// 011 = x4
		// 100 = x8
		// 101 and above = x16
		unsigned int osrs_t : 3;   ///< temperature oversampling

		// pressure oversampling
		// 000 = skipped
		// 001 = x1
		// 010 = x2
		// 011 = x4
		// 100 = x8
		// 101 and above = x16
		unsigned int osrs_p : 3;   ///< pressure oversampling

		// device mode
		// 00       = sleep
		// 01 or 10 = forced
		// 11       = normal
		unsigned int mode : 2;   ///< device mode

		/// @return combined ctrl register
		auto get() -> unsigned int {
			return (osrs_t << 5) | (osrs_p << 2) | mode;
		}
	};
	ctrl_meas_s m_measReg;   //!< measurement register object

	/**************************************************************************/
	/*!
		@brief  ctrl_hum register
	*/
	/**************************************************************************/
	struct ctrl_hum_s {
		/// unused - don't set
		unsigned int none : 5;

		// pressure oversampling
		// 000 = skipped
		// 001 = x1
		// 010 = x2
		// 011 = x4
		// 100 = x8
		// 101 and above = x16
		unsigned int osrs_h : 3;   ///< pressure oversampling

		/// @return combined ctrl hum register
		auto get() -> unsigned int {
			return (osrs_h);
		}
	};
	ctrl_hum_s m_humReg;   //!< hum register object
};