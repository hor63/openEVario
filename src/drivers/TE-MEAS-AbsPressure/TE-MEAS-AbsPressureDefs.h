/*
 * TE_MEAS_AbsPressure.h
 *
 *  Created on: Apr 26, 2021
 *      Author: hor
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2020  Kai Horstmann
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

#ifndef DRIVERS_TE_MEAS_AbsPressureDefs_H_
#define DRIVERS_TE_MEAS_AbsPressureDefs_H_

#include "CommonDefs.h"
#include "util/OEV_Enum.h"

/// Definitions for the TE_MEAS_AbsPressure sensor
/// \see [TE_MEAS_AbsPressure datasheet](https://www.nxp.com/docs/en/data-sheet/TE_MEAS_AbsPressure.pdf)

namespace openEV::drivers::TE_MEAS_AbsPressure {

	static constexpr uint8_t TE_MEAS_AbsPressureI2CAddr = 0x76;


	OEV_ENUM(TE_MEAS_AbsPressureCommands,
			CMD_Reset					= 0x1E,

			// Conversion of pressure
			CMD_Convert_D1_OSR_256		= 0x40,
			CMD_Convert_D1_OSR_512		= 0x42,
			CMD_Convert_D1_OSR_1024		= 0x44,
			CMD_Convert_D1_OSR_2048		= 0x46,
			CMD_Convert_D1_OSR_4096		= 0x48,

			// conversion of temperature
			CMD_Convert_D2_OSR_256		= 0x50,
			CMD_Convert_D2_OSR_512		= 0x52,
			CMD_Convert_D2_OSR_1024		= 0x54,
			CMD_Convert_D2_OSR_2048		= 0x56,
			CMD_Convert_D2_OSR_4096		= 0x58,

			CMD_ADC_Read				= 0x00,

			CMD_PROM_Read_Base			= 0xA0
	);

	uint8_t static inline CMD_PROM_READ_REG (uint8_t reg) {
		return CMD_PROM_Read_Base | (reg & 0b00000111)<<1;
	}

	OEV_ENUM(TE_MEAS_AbsPressurePROMRegs,
			/// Manufacturer reserved, only relevant for CRC calculation
			PROM_REG_RESERVED	= 0,
			PROM_REG_COEFF_1	= 1,
			PROM_REG_COEFF_2	= 2,
			PROM_REG_COEFF_3	= 3,
			PROM_REG_COEFF_4	= 4,
			PROM_REG_COEFF_5	= 5,
			PROM_REG_COEFF_6	= 6,
			/// The lower 4 bit contain the 4-bit CRC.
			/// The upper 4 bits may contain data which must be included in the CRC calculation
			/// Before the CRC calculation the lower CRC bits themselves must be set to 0.
			PROM_REG_CRC		= 7,
			/// Not a register, but the length of the register file
			PROM_REG_COUNT		= 8,
	);

	static constexpr uint8_t PROM_CRC_MASK = 0b00001111;

} // namespace openEV::drivers::TE_MEAS_AbsPressure

#endif /* DRIVERS_TE_MEAS_AbsPressureDefs_H_ */
