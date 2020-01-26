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

#include "util/io/SerialPort.h"

namespace openEV {
namespace io {


SerialPort::SerialPort(
		char const* portName,
		char const* portType
		) :
	StreamPort{portName,portType}
{
	// Do not let the tty become the controling terminal of the process. I am only using it as a binary comminucations channel.
	deviceOpenFlags |= O_NOCTTY;

}

SerialPort::~SerialPort() {
	// TODO Auto-generated destructor stub
}

} /* namespace io */
} /* namespace openEV */
