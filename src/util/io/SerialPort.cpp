/*
 * SerialPort.cpp
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
#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include "util/io/SerialPort.h"

#if HAVE_TERMIOS_H
#	include <termios.h>
#endif
#include <unistd.h>

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.IO.SerialPort");
	}
}

#endif


namespace openEV {
namespace io {

/** \brief Helper class to automatically register TCP ports with \ref PortBase
 *
 */
class SerialPortRegister {
private:

	SerialPortRegister() {
#if defined HAVE_LOG4CXX_H
		initLogger();
#endif /* HAVE_LOG4CXX_H */

		SerialPort::registerSerialPortType();
	}

	static SerialPortRegister theOneAndOnly;
};

SerialPortRegister SerialPortRegister::theOneAndOnly;

SerialPort::SerialPort(
		char const* portName
		) :
	StreamPort{portName,SerialPortType}
{
	// Do not let the tty become the controling terminal of the process. I am only using it as a binary comminucations channel.
	deviceOpenFlags |= O_NOCTTY;

}

SerialPort::~SerialPort() {

}

void SerialPort::openInternal() {
}

void SerialPort::configurePort(
		const Properties4CXX::Properties &globalConfiguration,
		const Properties4CXX::Properties &portConfiguration) {
}

PortBase* SerialPort::serialPortConstructor(const char *portName,
		const Properties4CXX::Properties &portProp) {
	return new SerialPort(portName);
}

void SerialPort::registerSerialPortType() {
	addPortType(SerialPortType,serialPortConstructor);
}


} /* namespace io */
} /* namespace openEV */
