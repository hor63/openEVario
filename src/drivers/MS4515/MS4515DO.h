/*
 * MS4515DO.h
 *
 *  Created on: Jan 17 2021
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

#ifndef DRIVERS_MS4515DO_H_
#define DRIVERS_MS4515DO_H_

#include "CommonDefs.h"
#include "util/OEV_Enum.h"

/// Definitions for the MS4515DO sensor
/// \see [MS4515DO Web site](https://www.te.com/global-en/product-CAT-BLPS0040.html) with links to the documents below and more.
/// \see [MS4515DO datasheet](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS4515DO%7FB10%7Fpdf%7FEnglish%7FENG_DS_MS4515DO_B10.pdf%7FCAT-BLPS0001)
/// \see [App note: Interfacing To MEAS Digital Pressure Modules](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Specification+Or+Standard%7FInterfacing_to_DigitalPressure_Modules%7FA3%7Fpdf%7FEnglish%7FENG_SS_Interfacing_to_DigitalPressure_Modules_A3.pdf%7FCAT-BLPS0001)
/// \see [App note: MS45XX SERIES](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Specification+Or+Standard%7FMS45xx_Application_Note%7FA1%7Fpdf%7FEnglish%7FENG_SS_MS45xx_Application_Note_A1.pdf%7FCAT-BLPS0001)
///  for PCB design for through-hole and SMT devices.

namespace openEV::drivers::MS4515 {

	/// Default I2C address of a MS4515
	static constexpr uint32_t MS4515DOI2CAddr = 0x46;

	/** Conversion between pressure in Inches of Water column (inH2) to mBar.
	 *
	 * Seriously, is *ANYONE* using crap units like this still these days???? \n
	 * And I am not talking about Imperial Units which are precisely defined to metric ones
	 * but ones which depend on material properties like here density of water (depending on temperature),
	 * and gravity of Earth. \n
	 * Do a Google search and you get a lot slightly different values on the last digits. \n
	 * Main reason seems that no one can agree on the temperature of the water in that column,
	 * and thus density. In addition the pressure value also depends on the local Gravity :D
	 *
	 * I cannot believe that TE Connectivity (formerly Measurement Specialties - MEAS Sensors)
	 * are using something so ill defined.
	 *
	 * End of rant
	 *
	 * \see [Wikipedia: Inch of water](https://en.wikipedia.org/wiki/Inch_of_water)
	 */
	static constexpr FloatType InchH2OtoMBar = 2.49082;

#if defined DOXYGEN
	enum MS4515Register {
#else
		OEV_ENUM(MS4515Register,
#endif
	 	MS4515_BRIDGE_HIGH			= 0x00,	///< Pressure data from the sensing bridge High-Bits, *and* status bits
		MS4515_BRIDGE_LOW			= 0x01,	///< Pressure data from the sensing bridge Low-byte
		MS4515_TEMP_HIGH			= 0x02,	///< High-byte of temperature data.
		MS4515_TEMP_LOW				= 0x03,	///< Low-bits of temperature data
#if defined DOXYGEN
    };
#else
    );
#endif

	static uint32_t constexpr MS4515_STATUS_BIT		= 6; ///< Number of bits to shift \ref MS4515_BRIDGE_HIGH byte data right
	static uint32_t constexpr MS4515_STATUS_MASK	= 0b11000000; ///< Bitmask for status

	static uint32_t constexpr MS4515_TEMPERATURE_HIGH_MASK	= ~MS4515_STATUS_MASK; ///< Bitmask for status

#if defined DOXYGEN
	enum MS4515Status {
#else
	OEV_ENUM(MS4515Status,
#endif
			MS4515_STATUS_OK			= 0,	///< Status OK, fresh data are in the data bytes.
			MS4515_STATUS_RESERVED		= 1,	///< Undefined status.
			MS4515_STATUS_STALE			= 2,	///< Valid data, but data were retrieved before. No new conversion occurred since the last read.
			MS4515_STATUS_FAULT			= 3,	///< Fault detection. Invalid data. Perform a power-on reset to get out of the funk.
#if defined DOXYGEN
    };
#else
    );
#endif


	/// Mask for high-byte of pressure data (Bridge data high). The upper bits are occupied by the status bits.
	static uint32_t constexpr MS4515_PRESSURE_HIGH_BYTE_MASK = ~MS4515_STATUS_MASK;

	/// The lower temperature bits are sitting in the *upper* bits of the \ref MS4515_TEMP_LOW byte
	static uint32_t constexpr MS4515_TEMPERATURE_LOW_BIT = 5;
	/// Mask of the low-byte temperature bits in the \ref MS4515_TEMP_LOW byte
	static uint32_t constexpr MS4515_TEMPERATURE_LOW_MASK = 0b11100000;

	/// Number of bits to shift \ref MS4515_TEMP_HIGH byte left before ORing with the low bits in \ref MS4515_TEMP_LOW byte
	static uint32_t constexpr MS4515_TEMPERATURE_HIGH_SHIFT = 8 - MS4515_TEMPERATURE_LOW_BIT;

} // namespace openEV::drivers::MS4515

std::ostream& operator << (std::ostream &o, openEV::drivers::MS4515::MS4515Register v);
std::ostream& operator << (std::ostream &o, openEV::drivers::MS4515::MS4515Status v);
#if defined HAVE_LOG4CXX_H
std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::drivers::MS4515::MS4515Register v);
std::ostream& operator << (log4cxx::helpers::CharMessageBuffer &b, openEV::drivers::MS4515::MS4515Status v);
#endif

#endif /* DRIVERS_MS4515DO_H_ */
