/*
 * DatagramPort.h
 *
 *  Created on: Aug 09, 2020
 *      Author: hor
 *
 *  Definition of class DatagramPort, the base class of datagram/message based ports
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

#ifndef UTIL_IO_DATAGRAMPORT_H_
#define UTIL_IO_DATAGRAMPORT_H_

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
class OEV_UTILS_PUBLIC DatagramPort: public PortBase {
protected:
	/**
	 *
	 * @param portName
	 * @param portType
	 *
	 * \see \ref PortBase::PortBase()
	 */
	DatagramPort(
			char const* portName,
			char const* portType
			);
public:
	virtual ~DatagramPort();

	/** \brief Read the next available message into the buffer
	 *
	 * The default implementation calls ::[recv()](https://man7.org/linux/man-pages/man2/recv.2.html)
	 * to read the next datagram into the provided \p buffer with \p bufLen bytes size
	 *
	 * Like all I/O operations access to the PortBase::deviceHandle is synchronized with an object of class \ref PortBase::DeviceHandleAccess
	 *
	 * When no datagram is available the call will block until data is available unless non-blocking mode is active
	 * with PortBase::isBlocking() = \a false. \n
	 * The function returns 0 when non-blocking mode is active, and no data is available. \n
	 *
	 * @param[out] buffer Buffer to receive the data
	 * @param[in] bufLen Size of the buffer. This is the maximum number bytes which can be received
	 * @return Number of bytes read into \p buffer.
	 * @throws GliderVarioPortReadException, GliderVarioPortNotOpenException GliderVarioPortLocalPortUndefined
	 *
	 * \see Linux Programmer's Manual: [read(2)](https://man7.org/linux/man-pages/man2/recv.2.html)
	 */
	virtual ssize_t recv(uint8_t * buffer,size_t bufLen);

	/** \brief write the buffer content at once as datagram to the peer address
	 *
	 * The default implementation calls ::[send()](http://man7.org/linux/man-pages/man2/send.2.html)
	 * to write the buffer content to the destination port/device/file...
	 *
	 * Note that send() is virtual and can be overridden.
	 *
	 * Like all I/O operations access to the device handle is synchronized with an object of class \ref PortBase::DeviceHandleAccess
	 *
	 * The call will block until data is written unless non-blocking mode is active
	 * with PortBase::isBlocking() = \a false. \n
	 *
	 * The function returns 0 when non-blocking mode is active, and no data can be written. \n
	 *
	 * @param buffer Buffer containing the data to be written
	 * @param bufLen Number of bytes in the buffer
	 * @return Number of bytes written. May be less then \p bufLen when the write call is interrupted or the destination full.
	 * @throws GliderVarioPortWriteException, GliderVarioPortNotOpenException, GliderVarioPortPeerPortUndefined,
	 * GliderVarioPortPeerAddressUndefined
	 *
	 * \see Linux Programmer's Manual: [send(2)](http://man7.org/linux/man-pages/man2/send.2.html)
	 * \see writeExactLen()
	 */
	virtual ssize_t send (uint8_t* buffer,size_t bufLen);

};

} /* namespace io */
} /* namespace openEV */

#endif /* UTIL_IO_DATAGRAMPORT_H_ */
