/*
 * StreamPort.h
 *
 *  Created on: Jul 16, 2019
 *      Author: hor
 *
 *  Definition of class PortBase, the base class of all IO ports
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

#ifndef UTIL_IO_STREAMPORT_H_
#define UTIL_IO_STREAMPORT_H_

#include "util/GliderVarioExceptionBase.h"
#include "util/io/PortBase.h"

namespace openEV {
namespace io {

/** \brief Base class for all byte-wise stream io port types
 *
 * Stream ports can be read and written byte-wise.
 * Reading and writing can be implemented block wise to and from a buffer,
 * but reading into a buffer does *not* imply that message boundaries or boundaries of block writes are being observed.
 *
 */
class OEV_UTILS_PUBLIC StreamPort: public PortBase {
protected:
	/**
	 *
	 * @param portName
	 * @param portType
	 *
	 * \see \ref PortBase::PortBase()
	 */
	StreamPort(
			char const* portName,
			char const* portType
			);
public:
	virtual ~StreamPort();

	/** \brief Read the next available block of data into the buffer
	 *
	 * The default implementation calls ::read() to read any available data up to bufLen bytes into the provided buffer
	 * When no data is available the call will block until data is available
	 *
	 * Like all I/O operations access to the \p deviceHandle is synchronized with an object of class \ref PortBase::DeviceHandleAccess
	 *
	 * @param buffer Buffer to receive the data
	 * @param bufLen Size of the buffer. This is the maximum number bytes which can be received
	 * @return Number of bytes read into \p buffer. Unless an exception was encountered
	 * @throws GliderVarioPortReadException
	 *
	 * \see Linux Programmer's Manual: [read(2)](http://man7.org/linux/man-pages/man2/read.2.html)
	 */
	virtual ssize_t read(uint8_t * buffer,size_t bufLen);

	/** \brief Read an exact amount of data
	 *
	 * This function calls read() in a loop until the requested number of bytes \p bufLen
	 * have been read. Note that read() is virtual and can be overridden.
	 *
	 * Like all I/O operations access to the \p deviceHandle is synchronized with an object of class \ref PortBase::DeviceHandleAccess
	 *
	 * @param buffer Buffer to receive the data
	 * @param bufLen Size of the buffer. This is the target number bytes to be received
	 * @return Number of bytes read into \p buffer. Unless an exception was thrown the function always returns \p bufLen
	 * @throws GliderVarioPortReadException
	 *
	 * \see StreamPort::read()
	 */
	ssize_t readExactLen(uint8_t* buffer,size_t bufLen);

	/** \brief write the buffer content to the port device
	 *
	 * The default implementation calls ::write() to write the entire buffer to the destination port/device/file...
	 * Data is written in a loop if only a part of the buffer is written until the entire buffer is written unless an error occurs.
	 *
	 * Like all I/O operations access to the device handle is synchronized with an object of class \ref PortBase::DeviceHandleAccess
	 *
	 * @param buffer Buffer containing the data to be written
	 * @param bufLen Number of bytes in the buffer
	 * @return Number of bytes written. Is always the same as \p bufLen. Else an error occurred and an exception is thrown.
	 * @throws GliderVarioPortWriteException
	 *
	 * \see Linux Programmer's Manual: [read(2)](http://man7.org/linux/man-pages/man2/write.2.html)
	 */
	virtual ssize_t write (uint8_t* buffer,size_t bufLen);

	/** \brief write an exact amount of data
	 *
	 * This function calls write() in a loop until the requested number of bytes \p bufLen
	 * have been written. Note that write() is virtual and can be overridden.
	 *
	 * Like all I/O operations access to the \p deviceHandle is synchronized with an object of class \ref PortBase::DeviceHandleAccess
	 *
	 * @param buffer Buffer containing the data to be sent
	 * @param bufLen Size of the buffer. This is the target number bytes to be sent
	 * @return Number of bytes sent. Unless an exception was thrown the function always returns \p bufLen
	 * @throws GliderVarioPortWriteException
	 *
	 * \see StreamPort::write()
	 */
	ssize_t writeExactLen(uint8_t* buffer,size_t bufLen);

};

} /* namespace io */
} /* namespace openEV */

#endif /* UTIL_IO_STREAMPORT_H_ */
