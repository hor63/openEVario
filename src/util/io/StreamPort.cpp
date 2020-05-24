/*
 * StreamPort.cpp
 *
 *  Created on: Jul 16, 2019
 *      Author: hor
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

#include <sstream>

#include "util/io/StreamPort.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.IO.PortBase");
	}
}

#endif

namespace openEV {
namespace io {

StreamPort::StreamPort(
		char const* portName,
		char const* portType
		) :
		PortBase {portName,portType}
{
	streamPort = true;
	initLogger();
}

StreamPort::~StreamPort() {

}

ssize_t StreamPort::read(uint8_t* buffer,size_t bufLen) {
	ssize_t ret;
	int err;
	DeviceHandleAccess devHandleAccess (*this);

	do {
		err = 0;

		ret = ::read(devHandleAccess.deviceHandle,buffer,bufLen);

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

				str << "Port" << getPortName() << ':' << getPortType() << ": Read error " << err << ":" << strerror(err);
				LOG4CXX_ERROR (logger,str.str());
				throw GliderVarioPortReadException(__FILE__,__LINE__,str.str().c_str(),err);
			}
		} else if (ret == 0) {
			// End-of-file condition
			std::ostringstream str;

			close();

			str << "Port" << getPortName() << ':' << getPortType() << ": End of file condition upon read";
			LOG4CXX_ERROR (logger,str.str());
			throw GliderVarioPortReadEndOfFileException(__FILE__,__LINE__,str.str().c_str());
		}

	} while (err == EINTR);

	LOG4CXX_DEBUG(logger,"StreamPort::read: ret = " << ret);

	return ret;
}

ssize_t StreamPort::write(uint8_t *buffer, size_t bufLen) {
	ssize_t ret;
	int err;
	DeviceHandleAccess devHandleAccess (*this);
	ssize_t bytesWritten = 0;

	do {
		err = 0;

		ret = ::write(devHandleAccess.deviceHandle,buffer,bufLen);

		if (ret == -1) {
			err = errno;
			if (err == EINTR) {
				LOG4CXX_DEBUG (logger,"Port" << getPortName() << ':' << getPortType() << ": Read interrupted with EINTR. Repeat ::read() ");
			} else {
				std::ostringstream str;
				str << "Port" << getPortName() << ':' << getPortType() << ": Write error " << err << ":" << strerror(err);
				LOG4CXX_ERROR (logger,str.str());

				throw GliderVarioPortWriteException(__FILE__,__LINE__,str.str().c_str());
			}

			switch (err) {

			case EINTR:
				LOG4CXX_DEBUG (logger,"Port" << getPortName() << ':' << getPortType() << ": Write interrupted with EINTR. Repeat ::read() ");
				break;

			case EWOULDBLOCK:
#if EWOULDBLOCK != EAGAIN
			case EAGAIN:
#endif
				ret = 0;
				break;
			default:
				std::ostringstream str;

				str << "Port" << getPortName() << ':' << getPortType() << ": Write error " << err << ":" << strerror(err);
				LOG4CXX_ERROR (logger,str.str());
				throw GliderVarioPortWriteException(__FILE__,__LINE__,str.str().c_str(),err);
			}
		} else if (ret == 0) {
			// End-of-file condition or undefined condition
			err = errno;
			std::ostringstream str;

			close();

			str << "Port" << getPortName() << ':' << getPortType() << ": Write returned 0. Error = " << err << ":" << strerror(err);
			LOG4CXX_ERROR (logger,str.str());
			throw GliderVarioPortReadEndOfFileException(__FILE__,__LINE__,str.str().c_str());
		}

	} while (err == EINTR);

	LOG4CXX_DEBUG(logger,"StreamPort::write: ret = " << ret);

	return ret;
}

ssize_t StreamPort::readExactLen(uint8_t *buffer, size_t bufLen) {
	ssize_t rc = read(buffer,bufLen);
	ssize_t bytesRead = 0;

	while (rc > 0 ) {
		bytesRead += rc;
		if (bytesRead >= bufLen) {
			break;
		}
		buffer += rc;
		bufLen -= rc;
		rc = read(buffer,bufLen);
	}

	return bytesRead;
}

ssize_t StreamPort::writeExactLen(uint8_t *buffer, size_t bufLen) {
	ssize_t rc = write(buffer,bufLen);
	ssize_t bytesWritten = 0;

	while (rc > 0 ) {
		bytesWritten += rc;
		if (bytesWritten >= bufLen) {
			break;
		}
		buffer += rc;
		bufLen -= rc;
		rc = write(buffer,bufLen);
	}

	return bytesWritten;
}

} /* namespace io */
} /* namespace openEV */

