/*
 * TCPPort.h
 *
 *  Created on: Dec 23, 2019
 *      Author: hor
 *
 *  Definition of class TCPPort, the a TCP socket implementation for I/O ports
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

#ifndef UTIL_IO_TCPPORT_H_
#define UTIL_IO_TCPPORT_H_

#include "util/io/StreamPort.h"

namespace openEV {
namespace io {

class TCPPort: public StreamPort {
public:

	static constexpr char const* TcpPortType = "TCPClient";

	TCPPort(char const* portName);
	virtual ~TCPPort();

	virtual void openInternal() override;
	virtual void configurePort(
			const Properties4CXX::Properties &globalConfiguration,
			const Properties4CXX::Properties &portConfiguration) override;

	static PortBase* tcpPortConstructor(
			char const* portName,
			Properties4CXX::Properties const &portProp);
	static void registerTcpPortType() OEV_UTILS_PUBLIC;

private:

protected:
	std::string tcpAddr;
	std::string tcpPort;
};

} /* namespace io */
} /* namespace openEV */

#endif /* UTIL_IO_TCPPORT_H_ */
