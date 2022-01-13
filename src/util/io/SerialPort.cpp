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

SerialPort::LineSpeeds const SerialPort::lineSpeeds [] = {
			B0 , "0",
			B50 , "50",
			B75 , "75",
			B110 , "110",
			B134 , "134",
			B150 , "150",
			B200 , "200",
			B300 , "300",
			B600 , "600",
			B1200 , "1200",
			B1800 , "1800",
			B2400 , "2400",
			B4800 , "4800",
			B9600 , "9600",
			B19200 , "19200",
			B38400 , "38400",
#if defined B57600
			B57600 , "57600",
#endif
#if defined B115200
			B115200 , "115200",
#endif
#if defined B128000
			B128000 , "128000",
#endif
#if defined B230400
			B230400 , "230400",
#endif
#if defined B256000
			B256000 , "256000",
#endif
#if defined B460800
			B460800 , "460800",
#endif
#if defined B500000
			B500000 , "500000",
#endif
#if defined B576000
			B576000 , "576000",
#endif
#if defined B921600
			B921600 , "921600",
#endif
#if defined B1000000
			B1000000 , "1000000",
#endif
#if defined B1152000
			B1152000 , "1152000",
#endif
#if defined B1500000
			B1500000 , "1500000",
#endif
#if defined B2000000
			B2000000 , "2000000",
#endif
#if defined B2500000
			B2500000 , "2500000",
#endif
#if defined B3000000
			B3000000 , "3000000",
#endif
			// Here are some convenience abbreviations like in the old MS-DOS MODE command which
			// allowed serial speeds to be specified without the two trailing zeros. 9600 became 96
			// Like MODE COM1:96,n,8,1
			B110 , "11",
			B150 , "15",
			B300 , "30",
			B600 , "60",
			B1200 , "12",
			B2400 , "24",
			B4800 , "48",
			B9600 , "96",
			B19200 , "192",
			B38400 , "384",
#if defined B57600
			B57600 , "576",
#endif
#if defined B115200
			B115200 , "1152",
#endif

			// The end record to indicate the end of the array
			0,nullptr
	};

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
		errTxt << getPortName()<< ":" << getDeviceName() << " is not a TTY.";
		LOG4CXX_ERROR(logger,errTxt.str());
		throw GliderVarioPortIsNoTTYException (__FILE__,__LINE__,errTxt.str().c_str());
	}

	LOG4CXX_DEBUG(logger,getPortName() << " is a TTY. OK");

	memset (&tios,0,sizeof(tios));
	rc = ::tcgetattr(devHandleAcc.deviceHandle,&tios);
	if (rc != 0) {
		errTxt << "Port "<< getPortName() << ": Error tcgetattr: " << ::strerror(rc);
		LOG4CXX_ERROR(logger,errTxt.str());
		throw GliderVarioPortIsNoTTYException (__FILE__,__LINE__,errTxt.str().c_str());
	}
	LOG4CXX_DEBUG(logger,"Read struct termios for port " << getPortName());

#if defined HAVE_LOG4CXX_H
	if (logger->isDebugEnabled()) {
		printPortConfiguration();
	}
#endif

	// Some settings always required

	LOG4CXX_DEBUG(logger,"Set raw mode for port " << getPortName());
	cfmakeraw (&tios);

	// Setup the port to read blocking
	tios.c_cc[VMIN] = 1;
	tios.c_cc[VTIME] = 0;
	tios.c_cc[VSTART] = 021;
	tios.c_cc[VSTOP] = 023;


	// Flags to clear
	tios.c_iflag &= ~ (
			  IGNBRK	// Send \0 on BREAK condition, do not ignore
			| BRKINT	// Do not flush queues, do not send SIGINT to controlled program
			| IGNPAR	// Do not ignore framing and parity errors
			| ISTRIP	// Do not strip the 8th bit
			| INLCR		// Do not translate NL to CR
			| IGNCR		// Do not ignore CR on input
			| ICRNL		// Do not translate CR to NL
#if defined IUCLC
			| IUCLC		// Do not transform upper case characters to lower case
#endif
			| IXANY		// Disable typing any character will restart stopped output.
#if defined IMAXBEL
			| IMAXBEL	// Disable ring bell when input queue is full.
#endif
#if defined IUTF8
			| IUTF8		// Disable Input is UTF8
#endif
			);

	tios.c_oflag &= ~ (
			  OPOST		//Enable implementation-defined output processing.
#if defined OLCUC
			| OLCUC		// Disable Map lowercase characters to uppercase on output.
#endif
			| ONLCR		// Disable Map NL to CR-NL on output.
			| OCRNL		// Disable Map CR to NL on output.
			| ONOCR		// Don't output CR at column 0.
			| ONLRET 	// Do output CR.
			| OFILL		// Disable Send fill characters for a delay
			| OFDEL		// Disable Fill character is ASCII DEL (0177)
			| NLDLY		// Disable Newline delay mask
			| CRDLY		// Disable Carriage return delay mask
			| TABDLY	// Disable Horizontal tab delay mask.
			| BSDLY		// Disable Backspace delay mask.
			| VTDLY		// Disable Vertical tab delay mask.
			| FFDLY		// Disable Form feed delay mask.
			);

	// Clear control flags
	tios.c_cflag &= ~ (
			  CSIZE		// Character size mask. CS8 is set below
			| CSTOPB	// One stop bit
			| PARENB	// Enable parity generation on output and parity checking for input.
			| PARODD	// Disable odd parity
			| HUPCL		// Disable Lower modem control lines after last process closes the device
#if defined LOBLK
			| LOBLK		// Disable Block output from a noncurrent shell layer.
#endif
#if defined CMSPAR
			| CMSPAR	// Disable "stick" (mark/space) parity
#endif
			| CRTSCTS	// Disable RTS/CTS (hardware) flow control.
			);

	// Set control flags
	tios.c_cflag |= (
			  CS8		// 8 Bits
			| CREAD		// Enable receiver. This is a must on some devices. Others do not care.
			| CLOCAL	// Ignore modem control lines.
			);


	// Clear local flags
	tios.c_lflag &= ~ (
			  ISIG		// Disable signal generation
			| ICANON	// Disable canonical mode
#if defined XCASE
			| XCASE		// Disable uppercase mode
#endif
			| ECHO		// Disable Echo input characters.
			| ECHOE		// Disable ERASE and WERASE characters
			| ECHOK		// Disable KILL character
			| ECHONL	// Disable echo NL character
#if defined ECHOCTL
			| ECHOCTL	// Disable echo special characters X as ^X
#endif
#if defined ECHOPRT
			| ECHOPRT	// Disable printing deleted characters
#endif
#if defined ECHOKE
			| ECHOKE	// Disable some weird echoing upon KILL which I do not understand from the doc ;)
#endif
#if defined DEFECHO
			| DEFECHO	// Disable echo when a process is reading
#endif
#if defined FLUSHO
			| FLUSHO	// Disable flushing when the DISCARD character is being sent.
#endif
			| TOSTOP	// Disable Send the SIGTTOU signal
#if defined PENDIN
			| PENDIN	// Disable All characters in
						// the input queue are reprinted when the next character is read.
#endif
			| IEXTEN	// Disable implementation-defined input processing.
			);

	// Set local flags
	tios.c_lflag |= (
			NOFLSH	// Disable flushing the input and output queues when generating
					// signals for the INT, QUIT, and SUSP characters.

			);

	// Base and default settings are done.
	// Now set configuration settings.
	if (baud != B0) {
		cfsetspeed(&tios,baud);
	}

	tios.c_cflag =
			(tios.c_cflag & ~numBitsMask) // Clear the bits if the mask is set
			| (numBitsMask & numBits); // And set the bits, when the mask is set.

	tios.c_cflag =
			(tios.c_cflag & ~stopBitsMask) // Clear the bits if the mask is set
			| (stopBitsMask & stopBits); // And set the bits, when the mask is set.

	tios.c_cflag =
			(tios.c_cflag & ~parityMask) // Clear the bits if the mask is set
			| (parityMask & parity); // And set the bits, when the mask is set.

	tios.c_cflag =
			(tios.c_cflag & ~rtsCtsMask) // Clear the bits if the mask is set
			| (rtsCtsMask & rtsCts); // And set the bits, when the mask is set.

	tios.c_iflag =
			(tios.c_iflag & ~xonXoffMask) // Clear the bits if the mask is set
			| (xonXoffMask & xonXoff); // And set the bits, when the mask is set.


#if defined HAVE_LOG4CXX_H
	if (logger->isDebugEnabled()) {
		printPortConfiguration();
	}
#endif

	rc = ::tcsetattr(devHandleAcc.deviceHandle,TCSANOW,&tios);
	if (rc != 0) {
		errTxt << "Port "<< getPortName() << ": Error tcsetattr: " << ::strerror(rc);
		LOG4CXX_ERROR(logger,errTxt.str());
		throw GliderVarioPortIsNoTTYException (__FILE__,__LINE__,errTxt.str().c_str());
	}
	LOG4CXX_DEBUG(logger,"Set struct termios for port " << getPortName());

	// Re-read the configuration to obtain which changes actually took place
	memset (&tios,0,sizeof(tios));
	rc = ::tcgetattr(devHandleAcc.deviceHandle,&tios);
	if (rc != 0) {
		errTxt << "Port "<< getPortName() << ": Error tcgetattr: " << ::strerror(rc);
		LOG4CXX_ERROR(logger,errTxt.str());
		throw GliderVarioPortIsNoTTYException (__FILE__,__LINE__,errTxt.str().c_str());
	}
	LOG4CXX_DEBUG(logger,"Re-read struct termios for port " << getPortName());

#if defined HAVE_LOG4CXX_H
	if (logger->isDebugEnabled()) {
		printPortConfiguration();
	}
#endif

}

void SerialPort::configurePort(
		const Properties4CXX::Properties &globalConfiguration,
		const Properties4CXX::Properties &portConfiguration) {

	Properties4CXX::Property const * propVal;

	try {

		propVal = portConfiguration.searchProperty(baudPropertyName);
		LOG4CXX_DEBUG (logger,"Configure serial port \"" << getPortName() << ": baud rate = " << propVal->getStringValue());
		// Exceptions from this call will not be cought by the catch block below, what is exactly what I want.
		baud = getSpeedFromStr(propVal->getStrValue());
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

char const * SerialPort::getSpeedStr(speed_t speed) {
	char const *rc = "Unknown speed";

	for (int i = 0; lineSpeeds[i].speedStr != nullptr; i++) {
		if (speed == lineSpeeds[i].speed) {
			rc = lineSpeeds[i].speedStr;
			break;
		}
	}

	return rc;
}

speed_t SerialPort::getSpeedFromStr(const char *speedStr) {
	std::ostringstream errTxt;

	for (int i = 0; lineSpeeds[i].speedStr != nullptr; i++) {
		if (!strcmp(speedStr,lineSpeeds[i].speedStr)) {
			return lineSpeeds[i].speed;
		}
	}

	errTxt << "Error in " << __PRETTY_FUNCTION__ << ": Speed value \"" << speedStr << "\" is undefined";
	throw GliderVarioPortConfigException (__FILE__,__LINE__,errTxt.str().c_str());
}

static inline std::string printFlag(tcflag_t param,tcflag_t flag,char const *flagName) {
	std::ostringstream txt;

	txt << ' ';
	if ((param & flag) ==  flag ) {
		txt << '+';
	} else {
		txt << '-';
	}
	txt << flagName;

	return txt.str();
}

void SerialPort::printPortConfiguration() {

    tcflag_t const &iflag = tios.c_iflag;
    tcflag_t const &oflag = tios.c_oflag;
    tcflag_t const &cflag = tios.c_cflag;
    tcflag_t const &lflag = tios.c_lflag;
    std::ostringstream txt;

    LOG4CXX_DEBUG(logger,"Configuration of port" << getPortName());
	txt << "	iflag: "
			<< printFlag(iflag,IGNBRK,"IGNBRK")
			<< printFlag(iflag,BRKINT,"BRKINT")
			<< printFlag(iflag,IGNPAR,"IGNPAR")
			<< printFlag(iflag,PARMRK,"PARMRK")
			<< printFlag(iflag,INPCK,"INPCK")
			<< printFlag(iflag,ISTRIP,"ISTRIP")
			<< printFlag(iflag,INLCR,"INLCR")
			<< printFlag(iflag,IGNCR,"IGNCR")
			<< printFlag(iflag,ICRNL,"ICRNL")
#if defined IUCLC
			<< printFlag(iflag,IUCLC,"IUCLC")
#endif
			<< printFlag(iflag,IXON,"IXON")
			<< printFlag(iflag,IXANY,"IXANY")
			<< printFlag(iflag,IXOFF,"IXOFF")
#if defined IMAXBEL
			<< printFlag(iflag,IMAXBEL,"IMAXBEL")
#endif
#if defined IUTF8
			<< printFlag(iflag,IUTF8,"IUTF8")
#endif
			;
	LOG4CXX_DEBUG(logger,txt.str());

	txt.str("");
	txt << "	oflag: "
			<< printFlag(oflag,OPOST,"OPOST")
#if defined OLCUC
			<< printFlag(oflag,OLCUC,"OLCUC")
#endif
			<< printFlag(oflag,ONLCR,"ONLCR")
			<< printFlag(oflag,OCRNL,"OCRNL")
			<< printFlag(oflag,ONOCR,"ONOCR")
			<< printFlag(oflag,ONLRET,"ONLRET")
			<< printFlag(oflag,OFILL,"OFILL")
			<< printFlag(oflag,OFDEL,"OFDEL")
			<< printFlag(oflag,NL0,"NL0")
			<< printFlag(oflag,NL1,"NL1")
#if defined CRDLY
			<< printFlag(oflag,CR0,"CR0")
			<< printFlag(oflag,CR1,"CR1")
			<< printFlag(oflag,CR2,"CR2")
			<< printFlag(oflag,CR3,"CR3")
#endif
#if defined TABDLY
			<< printFlag(oflag,TAB0,"TAB0")
			<< printFlag(oflag,TAB1,"TAB1")
			<< printFlag(oflag,TAB2,"TAB2")
			<< printFlag(oflag,TAB3,"TAB3")
#endif
#if defined BSDLY
			<< printFlag(oflag,BS0,"BS0")
			<< printFlag(oflag,BS1,"BS1")
#endif
			<< printFlag(oflag,VT0,"VT0")
			<< printFlag(oflag,VT1,"VT1")
			<< printFlag(oflag,FF0,"FF0")
			<< printFlag(oflag,FF1,"FF1")
			;
	LOG4CXX_DEBUG(logger,txt.str());

	txt.str("");
	txt << "	cflag: "
			<< printFlag(cflag,CS5,"CS5")
			<< printFlag(cflag,CS6,"CS6")
			<< printFlag(cflag,CS7,"CS7")
			<< printFlag(cflag,CS8,"CS8")
			<< printFlag(cflag,CSTOPB,"CSTOPB")
			<< printFlag(cflag,CREAD,"CREAD")
			<< printFlag(cflag,PARENB,"PARENB")
			<< printFlag(cflag,PARODD,"PARODD")
			<< printFlag(cflag,HUPCL,"HUPCL")
			<< printFlag(cflag,CLOCAL,"CLOCAL")
#if defined LOBLK
			<< printFlag(cflag,LOBLK,"LOBLK")
#endif
#if defined CMSPAR
			<< printFlag(cflag,CMSPAR,"CMSPAR")
#endif
#if defined CRTSCTS
			<< printFlag(cflag,CRTSCTS,"CRTSCTS")
#endif
			;
	LOG4CXX_DEBUG(logger,txt.str());

	txt.str("");
	txt << "	lflag: "
			<< printFlag(lflag,ISIG,"ISIG")
			<< printFlag(lflag,ICANON,"ICANON")
#if defined XCASE
			<< printFlag(lflag,XCASE,"XCASE")
#endif
			<< printFlag(lflag,ECHO,"ECHO")
			<< printFlag(lflag,ECHOE,"ECHOE")
			<< printFlag(lflag,ECHOK,"ECHOK")
			<< printFlag(lflag,ECHONL,"ECHONL")
#if defined ECHOCTL
			<< printFlag(lflag,ECHOCTL,"ECHOCTL")
#endif
#if defined ECHOPRT
			<< printFlag(lflag,ECHOPRT,"ECHOPRT")
#endif
#if defined ECHOKE
			<< printFlag(lflag,ECHOKE,"ECHOKE")
#endif
#if defined DEFECHO
			<< printFlag(lflag,DEFECHO,"DEFECHO")
#endif
#if defined FLUSHO
			<< printFlag(lflag,FLUSHO,"FLUSHO")
#endif
			<< printFlag(lflag,NOFLSH,"NOFLSH")
			<< printFlag(lflag,TOSTOP,"TOSTOP")
#if defined PENDIN
			<< printFlag(lflag,PENDIN,"PENDIN")
#endif
			<< printFlag(lflag,IEXTEN,"IEXTEN")
			;
	LOG4CXX_DEBUG(logger,txt.str());

	txt.str("");
	txt << "	c_cc: "
			<< "VMIN = " << int(tios.c_cc[VMIN])
			<< " VTIME = " << int(tios.c_cc[VTIME])
			<< " VSTART = " << int(tios.c_cc[VSTART])
			<< " VSTOP = " << int(tios.c_cc[VSTOP])
			;
	LOG4CXX_DEBUG(logger,txt.str());

	LOG4CXX_DEBUG(logger,"Output speed = " << getSpeedStr(cfgetospeed(&tios)));
	LOG4CXX_DEBUG(logger,"Input speed = " << getSpeedStr(cfgetispeed(&tios)));

}

} /* namespace io */
} /* namespace openEV */
