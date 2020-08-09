/*
 * UDPPort.h
 *
 *  Created on: Aug 09, 2020
 *      Author: hor
 *
 *  Definition of class UDPPort, the a TCP socket implementation for I/O ports
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

#ifndef UTIL_IO_UDPPORT_H_
#define UTIL_IO_UDPPORT_H_

#include "util/io/DatagramPort.h"

namespace openEV {
namespace io {

class UDPPort: public DatagramPort {
public:

	static constexpr char const* UdpPortType = "UDP";

	UDPPort(char const* portName);
	virtual ~UDPPort();

	virtual void openInternal() override;
	virtual void configurePort(
			const Properties4CXX::Properties &globalConfiguration,
			const Properties4CXX::Properties &portConfiguration) override;

	static PortBase* udpPortConstructor(
			char const* portName,
			Properties4CXX::Properties const &portProp);
	static void registerTcpPortType();

private:

protected:
	std::string peerAddr;
	bool peerAddrDefined = false;
	std::string peerPort;
	bool peerPortDefined = false;
	std::string localAddr;
	bool localAddrDefined = false;
	std::string localPort;
	bool localPortDefined = false;

	/// true if the destination address and port are defined. \n
	/// Now you can send data
	bool socketConnected = false;
	/// true when the local address and port are defined. \n
	/// Now you can receive data
	bool socketBound = false;

};

} /* namespace io */
} /* namespace openEV */

#endif /* UTIL_IO_UDPPORT_H_ */
