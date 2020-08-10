/*
 * DatagramPort.cpp
 *
 *  Created on: Aug 09, 2020
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
 *
 */
#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <sys/types.h>
#include <sys/socket.h>
#include <sstream>

#include "util/io/DatagramPort.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.IO.DatagramPort");
	}
}

#endif

namespace openEV {
namespace io {

DatagramPort::DatagramPort(const char *portName, const char *portType)  :
				PortBase {portName,portType}
{
	datagramPort = true;
	initLogger();
}

DatagramPort::~DatagramPort() {
}

ssize_t DatagramPort::recv(uint8_t *buffer, size_t bufLen) {
	ssize_t ret;
	int err;
	DeviceHandleAccess devHandleAccess (*this);
	int flags = MSG_TRUNC;

	if (!isBlocking()) {
		flags |= MSG_DONTWAIT;
	}

	do {
		err = 0;

		ret = ::recv(devHandleAccess.deviceHandle,buffer,bufLen,flags);

		if (ret == -1) {
			err = errno;
			switch (err) {

			case EINTR:
				LOG4CXX_DEBUG (logger,"Port" << getPortName() << ':' << getPortType() << ": Read interrupted with EINTR. Repeat ::read() ");
				break;

			case EWOULDBLOCK:
#if EWOULDBLOCK != EAGAIN
			case EAGAIN:
#endif
				ret = 0;
				break;
			default:
				std::ostringstream str;

				str << "Port" << getPortName() << ':' << getPortType() << ": Recv error " << err << ":" << strerror(err);
				LOG4CXX_ERROR (logger,str.str());
				throw GliderVarioPortReadException(__FILE__,__LINE__,str.str().c_str(),err);
			}
		}

	} while (err == EINTR);

	if (ret > bufLen) {
		// The datagram was longer than the available buffer. This the message is truncated.
		// This is not acceptable for the sake of data integrity.
		LOG4CXX_ERROR(logger,"Port" << getPortName() << ':' << getPortType() << ": Buffer too small for the datagram."
				" Buffer length = " << bufLen << ", datagram length = " << ret);
		throw GliderVarioPortBufferTooSmallForDatagram(__FILE__,__LINE__);
	}

	LOG4CXX_DEBUG(logger,"DatagramPort::recv: ret = " << ret);

	return ret;

}

ssize_t DatagramPort::send(uint8_t *buffer, size_t bufLen) {
	ssize_t ret;
	int err;
	DeviceHandleAccess devHandleAccess (*this);
	ssize_t bytesWritten = 0;
	int flags = 0;

	if (!isBlocking()) {
		flags |= MSG_DONTWAIT;
	}

	do {
		err = 0;

		ret = ::send(devHandleAccess.deviceHandle,buffer,bufLen,flags);

		if (ret == -1) {
			err = errno;

			switch (err) {

			case EINTR:
				LOG4CXX_DEBUG (logger,"Port" << getPortName() << ':' << getPortType() << ": Write interrupted with EINTR. Repeat ::send() ");
				break;

			case EWOULDBLOCK:
#if EWOULDBLOCK != EAGAIN
			case EAGAIN:
#endif
				ret = 0;
				break;
			default:
				std::ostringstream str;

				str << "Port" << getPortName() << ':' << getPortType() << ": send error " << err << ":" << strerror(err);
				LOG4CXX_ERROR (logger,str.str());
				throw GliderVarioPortWriteException(__FILE__,__LINE__,str.str().c_str(),err);
			}
		}

	} while (err == EINTR);

	LOG4CXX_DEBUG(logger,"StreamPort::write: ret = " << ret);

	return ret;
}

} // namespace io
} // namespace openEV

