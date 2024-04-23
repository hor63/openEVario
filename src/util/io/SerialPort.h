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

#if HAVE_TERMIOS_H
#	include <termios.h>
#endif
#if HAVE_UNISTD_H
#	include <unistd.h>
#endif

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

	/**
	 * \brief Struct of the \ref lineSpeeds array
	 */
	struct LineSpeeds {
		speed_t speed; //< Speed as defined in B... constants in [termios(3)](https://man7.org/linux/man-pages/man3/termios.3.html)
		char const* speedStr; //< The speed as string
	};
	/**
	 * \brief Array of defined line speeds consisting of records of \ref LineSpeeds.
	 * The last entry marks the end of the array.
	 */
	static struct LineSpeeds const lineSpeeds [];

	SerialPort(
			char const* portName
			);
	virtual ~SerialPort();

	virtual void configurePort(
			const Properties4CXX::Properties &globalConfiguration,
			const Properties4CXX::Properties &portConfiguration) override;

	/** \brief Access the internal port configuration.
	 *
	 * @return Reference to the internal termios structure
	 * which is used to setup the port.
	 *
	 * \see [termios(3)](https://man7.org/linux/man-pages/man3/termios.3.html)
	 */
	struct termios const &getTermios() const {
		return tios;
	}

	/** \brief Get the line speed of a serial port as printable string
	 *
	 * @param speed Bxxx constant defined in [termios(3)](https://man7.org/linux/man-pages/man3/termios.3.html)
	 * @return The speed as string.
	 */
	static char const *getSpeedStr(speed_t speed);

	/** Get the line speed as Bxxx constant from a string with a speed value
	 *
	 * Defined speed values are defined in \ref lineSpeeds
	 * Throws a \ref GliderVarioPortConfigException when \p speedStr does not match a valid speed value.
	 *
	 * @param speedStr The speed as string. Examples are 1200, 2400, 4800...
	 * @return Bxxx constant defined in [termios(3)](https://man7.org/linux/man-pages/man3/termios.3.html)
	 * @throws GliderVarioPortConfigException
	 */
	speed_t getSpeedFromStr(char const * speedStr);

	/** \brief Static member function which constructs a SerialPort object
	 *
	 * @param portName Name of the new serial port
	 * @param portProp Properties structure of the serial port
	 * @return
	 */
	static PortBase* serialPortConstructor(
			char const* portName,
			Properties4CXX::Properties const &portProp);
	/**
	 * Register \ref StreamPort of type name \ref SerialPortType ("serial") calling PortBase::addPortType()
	 */
	static void registerSerialPortType() OEV_UTILS_PUBLIC;

protected:

	speed_t baud = B0; ///< Line speed in and out; Default keep settings

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

	/** \brief Device specific open method
	 *
	 * Open a TTY, i.e. a serial port. \n
	 * Setup the TTY, at least set it to raw mode,
	 * and set the parameters from the configuration,
	 * like speed, parity, flow control...
	 *
	 * \see PortBase::openInteral
	 */
	virtual void openInternal() override;

	/** \brief Setup the port during \ref openInternal() with the settings from \ref configurePort().
	 *
	 */
	void setupPort();

	/** \brief Print the port configuration to the logger when LOG4CXX Debug mode is on.
	 *
	 */
	void printPortConfiguration();

private:

	struct termios tios;

};

} /* namespace io */
} /* namespace openEV */

#endif /* UTIL_IO_SERIALPORT_H_ */
