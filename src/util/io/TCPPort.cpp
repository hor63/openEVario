/*
 * TCPPort.cpp
 *
 *  Created on: Dec 23, 2019
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

#if HAVE_SYS_TYPES_H
#	include <sys/types.h>
#endif

#if HAVE_SYS_SOCKET_H
#	include <sys/socket.h>
#endif

#if HAVE_NETDB_H
#	include <netdb.h>
#endif

#include "util/io/TCPPort.h"

namespace openEV {
namespace io {

/// (mandatory) Can be numeric or symbolic. IPV4 and IPV6 are supported.
static std::string const hostPropertyName = "host";

/// (mandatory) Numeric or symbolic port numbers (/etc/services) are supported.
static std::string const portPropertyName = "port";


/** \brief Helper class to automatically register TCP ports with \ref PortBase
 *
 */
class TCPPortRegister {
private:

	TCPPortRegister() {
		TCPPort::registerTcpPortType();
	}

	static TCPPortRegister theOneAndOnly;
};

TCPPortRegister TCPPortRegister::theOneAndOnly;

TCPPort::TCPPort(char const* portName)
	: StreamPort(portName,TcpPortType)
{

}

TCPPort::~TCPPort() {

}

PortBase* TCPPort::tcpPortConstructor(
		char const* portName,
		Properties4CXX::Properties const &portProp) {

	return new TCPPort(portName);
}

void TCPPort::registerTcpPortType() {
	addPortType(TcpPortType,tcpPortConstructor);
}

void TCPPort::configurePort(
		const Properties4CXX::Properties &globalConfiguration,
		const Properties4CXX::Properties &portConfiguration) {

	auto prop = portConfiguration.searchProperty(hostPropertyName);
	tcpAddr = prop->getStringValue();

	prop = portConfiguration.searchProperty(portPropertyName);
	tcpPort = prop->getStringValue();


}

void TCPPort::open() {
}

} /* namespace io */
} /* namespace openEV */
