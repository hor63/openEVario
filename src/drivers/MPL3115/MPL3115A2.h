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

//#include <stdint.h>

#include <OEVCommon.h>

/// Definitions for the MPL3115A2 sensor
/// \see [MPL3115A2 datasheet](https://www.nxp.com/docs/en/data-sheet/MPL3115A2.pdf)

namespace openEV::drivers::MPL3115 {

	uint8_t constexpr MPL3115A2I2CAddr = 0x60;

	/*
	 *
00h STATUS Sensor status register
01h OUT_P_MSB Pressure data out MSB; Bits 12 to 19 of 20-bit real-time pressure sample.
				Root pointer to pressure and temperature FIFO data.
02h OUT_P_CSB Pressure data out CSB; Bits 4 to 11 of 20-bit real-time pressure sample
03h OUT_P_LSB Pressure data out LSB; Bits 0 to 3 of 20-bit real-time pressure sample
04h OUT_T_MSB R 00h Temperature data out MSB; Bits 4 to 11 of 12-bit real-time temperature sample
05h OUT_T_LSB Temperature data out LSB; Bits 0 to 3 of 12-bit real-time temperature sample

0Fh F_SETUP FIFO setup register; FIFO setup

0Ch WHO_AM_I Device identification register; Fixed device ID number

11h SYSMOD System mode register; Current system mode
12h INT_SOURCE Interrupt source register; Interrupt status
13h PT_DATA_CFG PT data configuration register; Data event flag configuration

26h CTRL_REG1 Control register 1 [1][4] No Modes, oversampling 27h Section 14.22.1
27h CTRL_REG2 Control register 2 [1] No Acquisition time step 28h Section 14.22.2
28h CTRL_REG3 Control register 3 [1][4] No Interrupt pin configuration 29h Section 14.22.3
29h CTRL_REG4 Control register 4 [1][4] No Interrupt enables 2Ah Section 14.22.4
2Ah CTRL_REG5 Control register 5 [1][4] No Interrupt output pin assignment 2Bh Section 14.22.5

2Bh OFF_P Pressure data user offset register; Pressure data offset
2Ch OFF_T Temperature data user offset register; Temperature data offset
2Dh OFF_H Altitude data user offset register; Altitude data offset

	 */

} // namespace openEV::drivers::MPL3115

#endif /* DRIVERS_MPL3115_MPL3315A2_H_ */
