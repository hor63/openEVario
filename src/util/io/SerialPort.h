/*
 * SerialPort.h
 *
 *  Created on: Aug 5, 2019
 *      Author: hor
 *
 *  Definition of class SerialPort, a end point class representing a serial/UART connection
 *
 *   This file is part of openEVario, an electronic variometer for glider planes
 *   Copyright (C) 2019  Kai Horstmann
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
 *
 */

#ifndef UTIL_IO_SERIALPORT_H_
#define UTIL_IO_SERIALPORT_H_

#include "util/io/StreamPort.h"

#include <termios.h>
#include <unistd.h>

namespace openEV {
namespace io {

/** \brief Implementation of serial line/UART connections
 *
 * Specific to this class is the ability to configure the serial line parameters
 * Basically without configuration of the line parameters the serial port is opened
 * as a binary connection without anz data control.
 * However all line parameters like speed, data and stop bits and parity are left untouched.
 */
class SerialPort: public StreamPort {
public:

	static constexpr char const* SerialPortType = "serial";

	SerialPort(
			char const* portName
			);
	virtual ~SerialPort();

	virtual void openInternal() override;
	virtual void configurePort(
			const Properties4CXX::Properties &globalConfiguration,
			const Properties4CXX::Properties &portConfiguration) override;

	static PortBase* serialPortConstructor(
			char const* portName,
			Properties4CXX::Properties const &portProp);
	static void registerSerialPortType() OEV_UTILS_PUBLIC;

protected:

	speed_t baud = B0; ///< Line speed in and out; Default keep setting

	// c_cflag bits
	tcflag_t numBits = CS8; ///< Number of data bits
	tcflag_t numBitsMask = 0; ///< Mask to activate \ref numBits. \\n 0 when inactive \n CSIZE when active

	tcflag_t stopBits = 0; ///< Number of stop bits Default 1 stop bit CSTOPB = 2 stop bits
	tcflag_t stopBitsMask = 0; ///< Mask to activate \ref stopBits. \n \\0 when inactive \n CSTOPB when active

	tcflag_t parity = 0; ///< Parity. Default no parity else PARENB|PARODD=odd, PARENB=even
	tcflag_t parityMask = 0; ///< Mask to activate \ref parity. \n \\0 when inactive \n PARENB|PARODD when active

	tcflag_t rtsCts = 0; ///< RTS/CTS flow control. Default off. CRTSCTS
	tcflag_t rtsCtsMask = 0; ///< Mask to activate \ref rtsCts. \n \\0 when inactive. CRTSCTS when active

	// c_iflag bits
	tcflag_t xonXoff = 0; ///< XON/XOFF flow control. Default off. IXON|IXOFF
	tcflag_t xonXoffMask = 0; ///< Mask to activate \ref xonXoff. \n \\0 when inactive \n IXON|IXOFF when active

};

} /* namespace io */
} /* namespace openEV */

#endif /* UTIL_IO_SERIALPORT_H_ */
