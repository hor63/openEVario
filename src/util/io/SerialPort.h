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
	static void registerSerialPortType();


};

} /* namespace io */
} /* namespace openEV */

#endif /* UTIL_IO_SERIALPORT_H_ */
