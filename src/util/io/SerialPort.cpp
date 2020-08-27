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

#include <sstream>

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.IO.SerialPort");
	}
}

#endif

#if !HAVE_CFMAKERAW || defined DOXYGEN
/** \brief Mimic the non-standard function cfmakeraw when it is not available
 *
 * @param termios_p Pointer to the termios structure
 *
 * \see Settings are verbatim from the description of \p cfmakeraw in [termios(3)](https://man7.org/linux/man-pages/man3/termios.3.html)
 */
static void cfmakeraw(struct termios *termios_p){
	termios_p->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
			| INLCR | IGNCR | ICRNL | IXON);
	termios_p->c_oflag &= ~OPOST;
	termios_p->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	termios_p->c_cflag &= ~(CSIZE | PARENB);
	termios_p->c_cflag |= CS8;

}
#endif

#if !HAVE_CFSETSPEED || defined DOXYGEN
/**
 *
 * @param[in,out] termios_p termios structure where the speed is written to
 * @param speed Speed of the interface. Constants see \p cfsetospeed() in
 * [termios(3)](https://man7.org/linux/man-pages/man3/termios.3.html)
 * @return 0 if OK, else \p errno bears the error.
 *
 * \see [termios(3)](https://man7.org/linux/man-pages/man3/termios.3.html)
 * \see [errno(3)](https://man7.org/linux/man-pages/man3/errno.3.html)
 */
static int cfsetspeed(struct termios *termios_p, speed_t speed) {
	int rc;

	rc = cfsetispeed(termios_p,speed);
	if (rc == 0) {
		rc = cfsetospeed(termios_p,speed);
	}

	return rc;
}
#endif



namespace openEV {
namespace io {

// Names of serial port configuration properties
static std::string const baudPropertyName = "baud";
static std::string const bitsPropertyName = "bits";
static std::string const stopBitsPropertyName = "stopbits";
static std::string const parityPropertyName = "parity";
static std::string const handshakePropertyName = "handshake";

SerialPort::SerialPort(
		char const* portName
		) :
	StreamPort{portName,SerialPortType}
{
#if defined HAVE_LOG4CXX_H
	initLogger();
#endif
	// Do not let the tty become the controling terminal of the process. I am only using it as a binary comminucations channel.
	deviceOpenFlags |= O_NOCTTY;

	memset (&tios,0,sizeof(tios));
}

SerialPort::~SerialPort() {

}

void SerialPort::openInternal() {

	// Use the generic open method of the base classes
	StreamPort::openInternal();
	// No error checking. If the device cannot be opened it throws an exception.

	setupPort();
}

void SerialPort::setupPort() {
	std::ostringstream errTxt;
	DeviceHandleAccess devHandleAcc (*this);
	int rc;

	LOG4CXX_DEBUG(logger,"Setup serial port \"" << getPortName());

	rc = ::isatty(devHandleAcc.deviceHandle);
	if (rc != 1) {
		errTxt << getPortName() << " is not a TTY: " << ::strerror(rc);
		LOG4CXX_ERROR(logger,errTxt.str());
		throw GliderVarioPortIsNoTTY (__FILE__,__LINE__,errTxt.str().c_str());
	}
	LOG4CXX_DEBUG(logger,getPortName() << " is a TTY. OK");

	rc = ::tcgetattr(devHandleAcc.deviceHandle,&tios);
	if (rc != 0) {
		errTxt << "Port "<< getPortName() << ": Error tcgetattr: " << ::strerror(rc);
		LOG4CXX_ERROR(logger,errTxt.str());
		throw GliderVarioPortIsNoTTY (__FILE__,__LINE__,errTxt.str().c_str());
	}
	LOG4CXX_DEBUG(logger,"Read struct termios for port " << getPortName());

	cfmakeraw (&tios);
}

void SerialPort::configurePort(
		const Properties4CXX::Properties &globalConfiguration,
		const Properties4CXX::Properties &portConfiguration) {

	Properties4CXX::Property const * propVal;

	try {
		baud = B0;

		propVal = portConfiguration.searchProperty(baudPropertyName);
		auto configVal = propVal->getIntVal();
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": baud rate = " << propVal->getStringValue());

		switch (configVal) {
		case 1200:
		case 12:
			baud = B1200;
			break;
		case 2400:
		case 24:
			baud = B2400;
			break;
		case 4800:
		case 48:
			baud = B4800;
			break;
		case 9600:
		case 96:
			baud = B9600;
			break;
		case 19200:
		case 192:
			baud = B19200;
			break;
		case 38400:
		case 384:
			baud = B38400;
			break;
// Going beyond POSIX from here on
#if defined B57600
		case 57600:
		case 576:
			baud = B57600;
			break;
#endif
#if defined B115200
		case 115200:
		case 1152:
			baud = B115200;
			break;
#endif
#if defined B230400
		case 230400:
			baud = B230400;
			break;
#endif
#if defined B460800
		case 460800:
			baud = B460800;
			break;
#endif
#if defined B500000
		case 500000:
			baud = B500000;
			break;
#endif
#if defined B576000
		case 576000:
			baud = B576000;
			break;
#endif
#if defined B921600
		case 921600:
			baud = B921600;
			break;
#endif
#if defined B1000000
		case 1000000:
			baud = B1000000;
			break;
#endif
		default:
			std::ostringstream errTxt;
			errTxt << "Configure serial port \"" << getPortName() << ": Baud rate "<< propVal->getStringValue() <<
					" is not recognized as a valid rate.";
			LOG4CXX_ERROR(logger,errTxt.str());
			throw GliderVarioPortConfigException (__FILE__,__LINE__,errTxt.str().c_str());
		}
	}
	catch (Properties4CXX::ExceptionPropertyNotFound const&e) {
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": No baud rate specified. Keep existing setting.");
	}
	catch (Properties4CXX::ExceptionWrongPropertyType const& e) {
		std::ostringstream errTxt;
		errTxt << "Configure serial port \"" << getPortName() << ": Baud rate is not numeric: " << e.what();
		LOG4CXX_ERROR(logger,errTxt.str());
		throw GliderVarioPortConfigException (__FILE__,__LINE__,errTxt.str().c_str());
	}

	try {
		numBits = CS8;
		numBitsMask = 0;

		propVal = portConfiguration.searchProperty(bitsPropertyName);
		auto configVal = propVal->getIntVal();
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": number bits = " << propVal->getStringValue());

		switch (configVal) {
		case 7:
			numBits = CS7;
			numBitsMask = CSIZE;
			break;
		case 8:
			numBits = CS8;
			numBitsMask = CSIZE;
			break;
		default:
			std::ostringstream errTxt;
			errTxt << "Configure serial port \"" << getPortName() << ": Number bits "<< propVal->getStringValue() <<
					" is invalid.";
			LOG4CXX_ERROR(logger,errTxt.str());
			throw GliderVarioPortConfigException (__FILE__,__LINE__,errTxt.str().c_str());
		}

	}
	catch (Properties4CXX::ExceptionPropertyNotFound const&e) {
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": No bit number rate specified. Keep existing setting.");
	}
	catch (Properties4CXX::ExceptionWrongPropertyType const& e) {
		std::ostringstream errTxt;
		errTxt << "Configure serial port \"" << getPortName() << ": Number bits is not numeric: " << e.what();
		LOG4CXX_ERROR(logger,errTxt.str());
		throw GliderVarioPortConfigException (__FILE__,__LINE__,errTxt.str().c_str());
	}

	try {
		parity = 0;
		parityMask = 0;

		propVal = portConfiguration.searchProperty(parityPropertyName);
		auto configVal = propVal->getStringValue();
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": parity = " << propVal->getStringValue());

		if (configVal == "n" || configVal == "none") {
			parity = 0;
			parityMask = PARENB|PARODD;
		} else if (configVal == "e" || configVal == "even") {
			parity = PARENB;
			parityMask = PARENB|PARODD;
		} else if (configVal == "o" || configVal == "odd") {
			parity = PARENB|PARODD;
			parityMask = PARENB|PARODD;
		} else {
			std::ostringstream errTxt;
			errTxt << "Configure serial port \"" << getPortName() << ": Parity value "<< propVal->getStringValue() <<
					" is invalid.";
			LOG4CXX_ERROR(logger,errTxt.str());
			throw GliderVarioPortConfigException (__FILE__,__LINE__,errTxt.str().c_str());
		}

	}
	catch (Properties4CXX::ExceptionPropertyNotFound const&e) {
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": No parity specified. Keep existing setting.");
	}

	try {
		stopBits = 0;
		stopBitsMask = 0;

		propVal = portConfiguration.searchProperty(stopBitsPropertyName);
		auto configVal = propVal->getIntVal();
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": number stop bits = " << propVal->getStringValue());

		switch (configVal) {
		case 1:
			stopBits = 0;
			stopBitsMask = CSTOPB;
			break;
		case 2:
			stopBits = CSTOPB;
			stopBitsMask = CSTOPB;
			break;
		default:
			std::ostringstream errTxt;
			errTxt << "Configure serial port \"" << getPortName() << ": Number stop bits "<< propVal->getStringValue() <<
					" is invalid.";
			LOG4CXX_ERROR(logger,errTxt.str());
			throw GliderVarioPortConfigException (__FILE__,__LINE__,errTxt.str().c_str());
		}

	}
	catch (Properties4CXX::ExceptionPropertyNotFound const&e) {
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": No bit number rate specified. Keep existing setting.");
		stopBits = 0;
		stopBitsMask = 0;
	}
	catch (Properties4CXX::ExceptionWrongPropertyType const& e) {
		std::ostringstream errTxt;
		errTxt << "Configure serial port \"" << getPortName() << ": Number bits is not numeric: " << e.what();
		LOG4CXX_ERROR(logger,errTxt.str());
		throw GliderVarioPortConfigException (__FILE__,__LINE__,errTxt.str().c_str());
	}

	try {
		rtsCts = 0;
		rtsCtsMask = 0;

		xonXoff = 0;
		xonXoffMask = 0;

		propVal = portConfiguration.searchProperty(handshakePropertyName);
		auto configVal = propVal->getStringValue();
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": handshake = " << propVal->getStringValue());

		if (configVal == "n" || configVal == "none") {
			rtsCtsMask = CRTSCTS;
			xonXoffMask = IXON|IXOFF;
		} else if (configVal == "rtscts") {
			rtsCts = CRTSCTS;
			rtsCtsMask = CRTSCTS;
			xonXoffMask = IXON|IXOFF;
		} else if (configVal == "xonxoff") {
			rtsCtsMask = CRTSCTS;
			xonXoff = IXON|IXOFF;
			xonXoffMask = IXON|IXOFF;
		} else {
			std::ostringstream errTxt;
			errTxt << "Configure serial port \"" << getPortName() << ": handshake value "<< propVal->getStringValue() <<
					" is invalid.";
			LOG4CXX_ERROR(logger,errTxt.str());
			throw GliderVarioPortConfigException (__FILE__,__LINE__,errTxt.str().c_str());
		}

	}
	catch (Properties4CXX::ExceptionPropertyNotFound const&e) {
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": No handshake specified. Keep existing setting.");
	}


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
