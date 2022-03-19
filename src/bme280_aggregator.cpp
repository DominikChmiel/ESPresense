/*!
 * Adapted from Adafruit_BME280.h + .cpp
 *  - Stipped out SPI support
 *  - Removed Unified sensor integrations
 *  - Changed readout to use single batch readout
 *  - Add history storage of samples in gRTC
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 * Adapted by Dominik Chmiel
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 * See the LICENSE file for details.
 *
 */

#include "bme280_aggregator.hpp"
#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <cstdint>


/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @param addr the I2C address the device can be found on
 *   @param theWire the I2C object to use, defaults to &Wire
 *   @returns true on success, false otherwise
 */
auto BME280Aggregator::begin(uint8_t addr, TwoWire* theWire) -> bool {
	bool status = false;
	m_i2caddr   = addr;
	m_wire      = theWire;
	status      = init();

	if (!status) {
		Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
		Serial.print("SensorID was: 0x");
		Serial.println(sensorID(), 16);
		Serial.println("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
		Serial.println("   ID of 0x56-0x58 represents a BMP 280,");
		Serial.println("        ID of 0x60 represents a BME 280.");
		Serial.println("        ID of 0x61 represents a BME 680.");
	}

	return status;
}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @returns true on success, false otherwise
 */
auto BME280Aggregator::init() -> bool {
	// I2C
	m_wire->begin();

	// check if sensor, i.e. the chip ID is correct
	m_sensorId = read8(BME280_REGISTER_CHIPID);
	if (m_sensorId != 0x60) {
		return false;
	}

	// reset the device using soft-reset
	// this makes sure the IIR is off, etc.
	write8(BME280_REGISTER_SOFTRESET, 0xB6);

	// wait for chip to wake up.
	delay(10);

	// if chip is still reading calibration, delay
	while (isReadingCalibration()) {
		delay(10);
	}

	readCoefficients();   // read trimming parameters, see DS 4.2.2

	// Change to lower sampling modes then default
	setSampling(MODE_NORMAL, SAMPLING_X8, SAMPLING_X4, SAMPLING_X4, FILTER_OFF, STANDBY_MS_0_5);

	delay(100);

	return true;
}

/*!
 *   @brief  setup sensor with given parameters / settings
 *
 *   This is simply a overload to the normal begin()-function, so SPI users
 *   don't get confused about the library requiring an address.
 *   @param mode the power mode to use for the sensor
 *   @param tempSampling the temp samping rate to use
 *   @param pressSampling the pressure sampling rate to use
 *   @param humSampling the humidity sampling rate to use
 *   @param filter the filter mode to use
 *   @param duration the standby duration to use
 */
void BME280Aggregator::setSampling(sensor_mode      mode,
								   sensor_sampling  tempSampling,
								   sensor_sampling  pressSampling,
								   sensor_sampling  humSampling,
								   sensor_filter    filter,
								   standby_duration duration) {
	m_measReg.mode   = mode;
	m_measReg.osrs_t = tempSampling;
	m_measReg.osrs_p = pressSampling;

	m_humReg.osrs_h    = humSampling;
	m_configReg.filter = filter;
	m_configReg.t_sb   = duration;

	// making sure sensor is in sleep mode before setting configuration
	// as it otherwise may be ignored
	write8(BME280_REGISTER_CONTROL, MODE_SLEEP);

	// you must make sure to also set REGISTER_CONTROL after setting the
	// CONTROLHUMID register, otherwise the values won't be applied (see
	// DS 5.4.3)
	write8(BME280_REGISTER_CONTROLHUMID, m_humReg.get());
	write8(BME280_REGISTER_CONFIG, m_configReg.get());
	write8(BME280_REGISTER_CONTROL, m_measReg.get());
}

/*!
 *   @brief  Writes an 8 bit value over I2C or SPI
 *   @param reg the register address to write to
 *   @param value the value to write to the register
 */
void BME280Aggregator::write8(byte reg, byte value) {
	m_wire->beginTransmission(m_i2caddr);
	m_wire->write(static_cast<uint8_t>(reg));
	m_wire->write(static_cast<uint8_t>(value));
	m_wire->endTransmission();
}

/*!
 *   @brief  Reads an 8 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the data byte read from the device
 */
auto BME280Aggregator::read8(byte reg) -> uint8_t {
	uint8_t value;

	m_wire->beginTransmission(m_i2caddr);
	m_wire->write(static_cast<uint8_t>(reg));
	m_wire->endTransmission();
	m_wire->requestFrom(m_i2caddr, static_cast<byte>(1));
	value = m_wire->read();
	return value;
}

/*!
 *   @brief  Reads a 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
auto BME280Aggregator::read16(byte reg) -> uint16_t {
	uint16_t value;

	m_wire->beginTransmission(m_i2caddr);
	m_wire->write(static_cast<uint8_t>(reg));
	m_wire->endTransmission();
	m_wire->requestFrom(m_i2caddr, static_cast<byte>(2));
	value = (m_wire->read() << 8) | m_wire->read();

	return value;
}

/*!
 *   @brief  Reads a signed 16 bit little endian value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
auto BME280Aggregator::read16_LE(byte reg) -> uint16_t {
	uint16_t temp = read16(reg);
	return (temp >> 8) | (temp << 8);
}

/*!
 *   @brief  Reads a signed 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
auto BME280Aggregator::readS16(byte reg) -> int16_t {
	return static_cast<int16_t>(read16(reg));
}

/*!
 *   @brief  Reads a signed little endian 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
auto BME280Aggregator::readS16_LE(byte reg) -> int16_t {
	return static_cast<int16_t>(read16_LE(reg));
}

/*!
 *   @brief  Reads the factory-set coefficients
 */
void BME280Aggregator::readCoefficients() {
	m_bme280Calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
	m_bme280Calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
	m_bme280Calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

	m_bme280Calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
	m_bme280Calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
	m_bme280Calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
	m_bme280Calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
	m_bme280Calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
	m_bme280Calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
	m_bme280Calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
	m_bme280Calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
	m_bme280Calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

	m_bme280Calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
	m_bme280Calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
	m_bme280Calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
	m_bme280Calib.dig_H4 = (static_cast<int8_t>(read8(BME280_REGISTER_DIG_H4)) << 4) | (read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
	m_bme280Calib.dig_H5 = (static_cast<int8_t>(read8(BME280_REGISTER_DIG_H5 + 1)) << 4) | (read8(BME280_REGISTER_DIG_H5) >> 4);
	m_bme280Calib.dig_H6 = static_cast<int8_t>(read8(BME280_REGISTER_DIG_H6));

#if 0
	LOGF("dig_T1: %10x | %10d\n", m_bme280Calib.dig_T1, m_bme280Calib.dig_T1);
	LOGF("dig_T2: %10x | %10d\n", m_bme280Calib.dig_T2, m_bme280Calib.dig_T2);
	LOGF("dig_T3: %10x | %10d\n", m_bme280Calib.dig_T3, m_bme280Calib.dig_T3);
	LOGF("dig_P1: %10x | %10d\n", m_bme280Calib.dig_P1, m_bme280Calib.dig_P1);
	LOGF("dig_P2: %10x | %10d\n", m_bme280Calib.dig_P2, m_bme280Calib.dig_P2);
	LOGF("dig_P3: %10x | %10d\n", m_bme280Calib.dig_P3, m_bme280Calib.dig_P3);
	LOGF("dig_P4: %10x | %10d\n", m_bme280Calib.dig_P4, m_bme280Calib.dig_P4);
	LOGF("dig_P5: %10x | %10d\n", m_bme280Calib.dig_P5, m_bme280Calib.dig_P5);
	LOGF("dig_P6: %10x | %10d\n", m_bme280Calib.dig_P6, m_bme280Calib.dig_P6);
	LOGF("dig_P7: %10x | %10d\n", m_bme280Calib.dig_P7, m_bme280Calib.dig_P7);
	LOGF("dig_P8: %10x | %10d\n", m_bme280Calib.dig_P8, m_bme280Calib.dig_P8);
	LOGF("dig_P9: %10x | %10d\n", m_bme280Calib.dig_P9, m_bme280Calib.dig_P9);
	LOGF("dig_H1: %10x | %10d\n", m_bme280Calib.dig_H1, m_bme280Calib.dig_H1);
	LOGF("dig_H2: %10x | %10d\n", m_bme280Calib.dig_H2, m_bme280Calib.dig_H2);
	LOGF("dig_H3: %10x | %10d\n", m_bme280Calib.dig_H3, m_bme280Calib.dig_H3);
	LOGF("dig_H4: %10x | %10d\n", m_bme280Calib.dig_H4, m_bme280Calib.dig_H4);
	LOGF("dig_H5: %10x | %10d\n", m_bme280Calib.dig_H5, m_bme280Calib.dig_H5);
	LOGF("dig_H6: %10x | %10d\n", m_bme280Calib.dig_H6, m_bme280Calib.dig_H6);
#endif
}

/*!
 *   @brief return true if chip is busy reading cal data
 *   @returns true if reading calibration, false otherwise
 */
auto BME280Aggregator::isReadingCalibration() -> bool {
	uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

	return (rStatus & (1 << 0)) != 0;
}

auto BME280Aggregator::adaptTemp(uint32_t raw_temp) -> int32_t {
	int32_t var1;
	int32_t var2;

	int32_t adc_T = raw_temp;
	if (adc_T == 0x800000) {   // value in case temp measurement was disabled
		return INT32_MAX;
	}
	adc_T >>= 4;

	var1 = ((((adc_T >> 3) - (static_cast<int32_t>(m_bme280Calib.dig_T1) << 1))) * (static_cast<int32_t>(m_bme280Calib.dig_T2))) >> 11;

	var2 = (((((adc_T >> 4) - (static_cast<int32_t>(m_bme280Calib.dig_T1))) * ((adc_T >> 4) - (static_cast<int32_t>(m_bme280Calib.dig_T1)))) >> 12) *
			(static_cast<int32_t>(m_bme280Calib.dig_T3))) >>
		   14;

	m_tFine = var1 + var2;

	return (m_tFine * 5 + 128);
};

auto BME280Aggregator::adaptPressure(uint32_t raw_press) -> int32_t {
	int64_t var1;
	int64_t var2;
	int64_t p;

	if (raw_press == 0x800000) {   // value in case pressure measurement was disabled
		return INT32_MAX;
	}
	raw_press >>= 4;

	var1 = (static_cast<int64_t>(m_tFine)) - 128000;
	var2 = var1 * var1 * static_cast<int64_t>(m_bme280Calib.dig_P6);
	var2 = var2 + ((var1 * static_cast<int64_t>(m_bme280Calib.dig_P5)) << 17);
	var2 = var2 + ((static_cast<int64_t>(m_bme280Calib.dig_P4)) << 35);
	var1 = ((var1 * var1 * static_cast<int64_t>(m_bme280Calib.dig_P3)) >> 8) + ((var1 * static_cast<int64_t>(m_bme280Calib.dig_P2)) << 12);
	var1 = ((((static_cast<int64_t>(1)) << 47) + var1)) * (static_cast<int64_t>(m_bme280Calib.dig_P1)) >> 33;

	if (var1 == 0) {
		return 0;   // avoid exception caused by division by zero
	}
	p    = 1048576 - raw_press;
	p    = (((p << 31) - var2) * 3125) / var1;
	var1 = ((static_cast<int64_t>(m_bme280Calib.dig_P9)) * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((static_cast<int64_t>(m_bme280Calib.dig_P8)) * p) >> 19;

	return ((p + var1 + var2) >> 8) + ((static_cast<int64_t>(m_bme280Calib.dig_P7)) << 4);
};

auto BME280Aggregator::adaptHumidity(uint32_t raw_humidity) -> int32_t {
	int32_t v_x1_u32r;

	if (raw_humidity == 0x8000) {   // value in case humidity measurement was disabled
		return INT32_MAX;
	}

	v_x1_u32r = (m_tFine - (static_cast<int32_t>(76800)));

	v_x1_u32r = (((((raw_humidity << 14) - ((static_cast<int32_t>(m_bme280Calib.dig_H4)) << 20) -
					((static_cast<int32_t>(m_bme280Calib.dig_H5)) * v_x1_u32r)) +
				   (static_cast<int32_t>(16384))) >>
				  15) *
				 (((((((v_x1_u32r * (static_cast<int32_t>(m_bme280Calib.dig_H6))) >> 10) *
					  (((v_x1_u32r * (static_cast<int32_t>(m_bme280Calib.dig_H3))) >> 11) + (static_cast<int32_t>(32768)))) >>
					 10) +
					(static_cast<int32_t>(2097152))) *
					   (static_cast<int32_t>(m_bme280Calib.dig_H2)) +
				   8192) >>
				  14));

	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * (static_cast<int32_t>(m_bme280Calib.dig_H1))) >> 4));

	v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
	v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
	return (v_x1_u32r >> 12);
};

auto BME280Aggregator::readAllSensors() -> sensor_data {
	// Datasheet MemoryMap has these in ascending order
	using sensor_regs = struct {
		uint8_t press_msb;
		uint8_t press_lsb;
		uint8_t press_xlsb;
		uint8_t temp_msb;
		uint8_t temp_lsb;
		uint8_t temp_xlsb;
		uint8_t hum_msb;
		uint8_t hum_lsb;

		auto getHum() -> uint32_t {
			return hum_msb << 8 | hum_lsb;
		};

		auto getTemp() -> uint32_t {
			return temp_msb << 16 | temp_lsb << 8 | temp_xlsb;
		};

		auto getPress() -> uint32_t {
			return press_msb << 16 | press_lsb << 8 | press_xlsb;
		};
	} __attribute__((packed));
	/* Read out 0xF7 to 0xFE */

	sensor_regs raw_regs;

	m_wire->beginTransmission(m_i2caddr);
	m_wire->write(static_cast<uint8_t>(BME280_REGISTER_PRESSUREDATA));
	m_wire->endTransmission();
	m_wire->requestFrom(m_i2caddr, static_cast<byte>(sizeof(raw_regs)));

	auto* raw_ptr = reinterpret_cast<uint8_t*>(&raw_regs);
	for (uint8_t i = 0; i < sizeof(raw_regs); i++) {
		*raw_ptr = m_wire->read();
		raw_ptr++;
	}

	return {
		.temperature = adaptTemp(raw_regs.getTemp()),
		.pressure    = adaptPressure(raw_regs.getPress()),
		.humidity    = adaptHumidity(raw_regs.getHum()),
	};
}

/*!
 *   Returns Sensor ID found by init() for diagnostics
 *   @returns Sensor ID 0x60 for BME280, 0x56, 0x57, 0x58 BMP280
 */
auto BME280Aggregator::sensorID() -> uint32_t {
	return m_sensorId;
}
