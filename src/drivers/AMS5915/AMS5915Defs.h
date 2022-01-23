/*
 * AMS5915.h
 *
 *  Created on: Apr 21 2021
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2021  Kai Horstmann
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License along
 *   with this program; if not, write to the Free Software Foundation, Inc.,
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef DRIVERS_AMS5915DEFS_H_
#define DRIVERS_AMS5915DEFS_H_

#include "CommonDefs.h"

/// Definitions for the AMS5915 sensor
/// \see [AMS5915 Web site](https://www.amsys-sensor.com/products/pressure-sensor/ams5915-digital-pressure-sensor-5mbar-to-10-bar/) with links to the documents below and more.
/// \see [AMS5915 datasheet](https://www.amsys-sensor.com/downloads/data/ams5915-AMSYS-datasheet.pdf)

namespace openEV::drivers::AMS5915 {

	/// Default I2C address of a AMS5915
	static constexpr uint32_t AMS5915I2CAddr = 0x28;


#if defined DOXYGEN
    enum AMS5915Register {
#else
	OEV_ENUM(AMS5915Register,
#endif
		AMS5915_PRESSURE_HIGH			= 0x00,	///< Pressure data from the sensing bridge High-Bits
		AMS5915_PRESSURE_LOW			= 0x01,	///< Pressure data from the sensing bridge Low-byte
		AMS5915_TEMP_HIGH				= 0x02,	///< High-byte of temperature data.
		AMS5915_TEMP_LOW				= 0x03,	///< Low-bits of temperature data
#if defined DOXYGEN
    };
#else
	);
#endif

	/// Bit mask (6 lower bits) of the high byte of the pressure registers
	static constexpr uint8_t AMS5915_PRESSURE_HIGH_BYTE_MASK = 0b00111111;

	/// Bit mask (upper 3 bits of the lower byte
	static constexpr uint8_t AMS5915_TEMP_LOW_BYTE_MASK = 0b11100000;
	/// Number of bits to shift the temperature value to the right.
	static constexpr uint8_t AMS5915_TEMP_SHIFT_COUNT = 5;

	/// Pressure register value at minimum measurement range
	static constexpr uint32_t AMS5915PressureRangeMinCount = 1638;

	/// Pressure register value at maximum measurement range
	static constexpr uint32_t AMS5915PressureRangeMaxCount = 14745;

} // namespace openEV::drivers::AMS5915

#endif /* DRIVERS_AMS5915DEFS_H_ */
