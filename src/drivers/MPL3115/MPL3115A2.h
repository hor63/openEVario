/*
 * MPL3115A2.h
 *
 *  Created on: Dec 28, 2020
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

#ifndef DRIVERS_MPL3115_MPL3315A2_H_
#define DRIVERS_MPL3115_MPL3315A2_H_

#include "CommonDefs.h"

/// Definitions for the MPL3115A2 sensor
/// \see [MPL3115A2 datasheet](https://www.nxp.com/docs/en/data-sheet/MPL3115A2.pdf)

namespace openEV::drivers::MPL3115 {

	static constexpr uint8_t MPL3115A2I2CAddr = 0x60;

	static constexpr uint8_t MPL3115WhoAmIValue = 0xC4;

#if defined DOXYGEN
    enum MPL3115Register {
#else
	OEV_ENUM(MPL3115Register,
#endif
		MPL3115_STATUS				= 0x00,	///< Sensor status register
		MPL3115_OUT_P_MSB			= 0x01,	///< Pressure data out MSB; Bits 12 to 19 of 20-bit real-time pressure sample. \n
											///< Root pointer to pressure and temperature FIFO data.
		MPL3115_OUT_P_CSB			= 0x02, ///< Pressure data out CSB; Bits 4 to 11 of 20-bit real-time pressure sample
		MPL3115_OUT_P_LSB			= 0x03, ///< Pressure data out LSB; Bits 0 to 3 of 20-bit real-time pressure sample
		MPL3115_OUT_T_MSB			= 0x04, ///< Temperature data out MSB; Bits 4 to 11 of 12-bit real-time temperature sample
		MPL3115_OUT_T_LSB			= 0x05, ///< Temperature data out LSB; Bits 0 to 3 of 12-bit real-time temperature sample

		MPL3115_F_SETUP				= 0x0F, ///< FIFO setup register; FIFO setup

		MPL3115_WHO_AM_I			= 0x0C, ///< Device identification register; Fixed device ID number

		MPL3115_SYSMOD				= 0x11, ///< System mode register; Current system mode
		MPL3115_INT_SOURCE			= 0x12, ///< Interrupt source register; Interrupt status
		MPL3115_PT_DATA_CFG			= 0x13, ///< PT data configuration register; Data event flag configuration

		MPL3115_CTRL_REG1			= 0x26, ///< Control register 1 Modes, oversampling
		MPL3115_CTRL_REG2			= 0x27, ///< Control register 2 Acquisition time step
		MPL3115_CTRL_REG3			= 0x28, ///< Control register 3 Interrupt pin configuration (Don't care)
		MPL3115_CTRL_REG4			= 0x29, ///< Control register 4 Interrupt enables (Not used, leave 0)
		MPL3115_CTRL_REG5			= 0x2A, ///< Control register 5 Interrupt output pin assignment (Don't care)

		MPL3115_OFF_P				= 0x2B, ///< Pressure data user offset register; Pressure data offset
		MPL3115_OFF_T				= 0x2C, ///< Temperature data user offset register; Temperature data offset
		MPL3115_OFF_H				= 0x2D, ///< Altitude data user offset register; Altitude data offset
#if defined DOXYGEN
    };
#else
	);
#endif

	enum MPL3115StatusBits {
		MPL3115Status_PTOW	= 7, ///< Pressure/altitude or temperature data overwrite.
		MPL3115Status_POW	= 6, ///< Pressure/altitude data overwrite
		MPL3115Status_TOW	= 5, ///< Temperature data overwrite
		MPL3115Status_PTDR	= 3, ///< Pressure/altitude or temperature data ready
		MPL3115Status_PDR	= 2, ///< New pressure/altitude data available
		MPL3115Status_TDR	= 1, ///< New temperature data available
	};

	enum MPL3115PTDataCfgBits {
		MPL3115PTDataCfg_DREM	= 2,	///< Data ready event mode.
						///< 0 — Event detection disabled (reset value) If the DREM bit is cleared logic '0' and one or
						///< more of the data ready event flags are enabled, then an event flag will be raised whenever
						///< the system acquires a new set of data.
						///< 1 — Generate data ready event flag on new pressure/altitude or temperature data. If the
						///< DREM bit is set logic '1' and one or more of the data ready event flags (PDEFE, TDEFE) are
						///< enabled, then an event flag will be raised upon change in state of the data.
		MPL3115PTDataCfg_PDEFE	= 1,	///< Data event flag enable on new pressure/altitude
						///< 0 — Event detection disabled (reset value)
						///< 1 — Raise event flag on new pressure/altitude data
		MPL3115PTDataCfg_TDEFE	= 0		///< Data event flag enable on new temperature data.
						///< 0 — Event detection disabled (reset value)
						///< 1 — Raise event flag on new temperature data
	};

	enum MPL3115Ctrl1Bits {
		MPL3115Cfg1_ALT		= 7, ///< Altitude (True), or barometric (false) mode
		MPL3115Cfg1_OS		= 3, ///< Oversample ratio. This is a 3-bit value (bit 3-5). \n
								 ///< The actual oversampling rate is actually 2^x.
		MPL3115Cfg1_RST		= 2, ///< Software reset
		MPL3115Cfg1_OST		= 1, ///< One-shot measurement immediately
		MPL3115Cfg1_SBYB	= 0, ///< Active (true)/Standby (false)
	};

	enum MPL3115Ctrl2Bits {
		MPL3115Cfg2_LoadOutput	= 5, ///< This is to load the target values for SRC_PW/SRC_TW and SRC_PTH/SRC_TTH (Not used here)
		MPL3115Cfg2_AlarmSel	= 4, ///< The bit selects the target value for SRC_PW/SRC_TW and SRC_PTH/SRC_TTH. (Don't care)
		MPL3115Cfg2_ST			= 0, ///< Auto acquisition time step. Auto acquisition cycle in sec. Bits 0-3. Actual value is 2^x
	};

} // namespace openEV::drivers::MPL3115

#endif /* DRIVERS_MPL3115_MPL3315A2_H_ */
