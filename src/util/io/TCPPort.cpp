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
#  include "config.h"
#endif

#include <cerrno>
#include <unistd.h>

#if HAVE_SYS_TYPES_H
#	include <sys/types.h>
#endif

#if HAVE_SYS_SOCKET_H
#	include <sys/socket.h>
#endif

#if HAVE_NETDB_H
#	include <netdb.h>
#endif

#if HAVE_NETINET_IN_H
#	include <netinet/in.h>
#endif

#if HAVE_NETINET_TCP_H
#	include <netinet/tcp.h>
#endif

#include <sstream>

#include "util/io/TCPPort.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.IO.TCPPort");
	}
}

#endif


namespace openEV {
namespace io {

/// (mandatory) Can be numeric or symbolic. IPV4 and IPV6 are supported.
static std::string const hostPropertyName = "host";

/// (mandatory) Numeric or symbolic port numbers (/etc/services) are supported.
static std::string const portPropertyName = "port";

TCPPort::TCPPort(char const* portName)
	: StreamPort(portName,TcpPortType)
{
#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	LOG4CXX_DEBUG(logger,__PRETTY_FUNCTION__
			<< "(portName=" << getPortName()
			<< ", portType=" << getPortType() << ')');

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

	LOG4CXX_DEBUG(logger,"Configure port " << getPortName()
			<< ": host = \"" << tcpAddr << '\"');

	prop = portConfiguration.searchProperty(portPropertyName);
	tcpPort = prop->getStringValue();

	LOG4CXX_DEBUG(logger,"Configure port " << getPortName()
			<< ": port = \"" << tcpPort << '\"');

}

void TCPPort::openInternal() {

	int sock = -1;
	int rc;
	struct addrinfo addrHint;
	struct addrinfo *addr = NULL;

	::memset (&addrHint,0,sizeof(addrHint));

	// Allow TCP as well as TCP V6; Do not insist on TCP V4
	// addrHint.ai_family = AF_INET;

	// But insist on TCP; do not allow UDP
	addrHint.ai_socktype = SOCK_STREAM;

	rc = ::getaddrinfo(tcpAddr.c_str(),tcpPort.c_str(),&addrHint,&addr);

	if (rc == 0) {
		struct addrinfo *ad = addr;

		while (ad) {

			auto sockType = ad->ai_socktype;

			if (!isBlocking()) {
				sockType |= SOCK_NONBLOCK;
			}

			sock = ::socket(ad->ai_family,sockType,ad->ai_protocol);

			if (sock == -1) {
				rc = errno;
				LOG4CXX_ERROR(logger,"Open port " << getPortName()
						<< ": socket() error: " << rc << '=' << strerror(rc));
				throw GliderVarioPortOpenException (
						__FILE__,
						__LINE__,
						"Error in socket()",
						rc);
			}

			if (sock != -1) {
				rc = ::connect(sock,ad->ai_addr,ad->ai_addrlen);
				if (rc == -1) {
					rc = errno;
					LOG4CXX_ERROR(logger,"Open port " << getPortName()
							<< ": connect() error: " << rc << '=' << strerror(rc));
					::close (sock);
					sock = -1;
					throw GliderVarioPortOpenException (
							__FILE__,
							__LINE__,
							"Error in connect()",
							rc);
				}
			}

			if (sock != -1) {
				break;
			}
			ad = ad->ai_next;
		}
	} else {
		std::ostringstream ostr;
		ostr << "getaddrinfo() error: " << gai_strerror(rc);
		LOG4CXX_ERROR(logger,"Open port " << getPortName()
				<< ostr.str());
		throw GliderVarioPortOpenException (
				__FILE__,
				__LINE__,
				ostr.str().c_str(),
				rc);
	}

	if (addr != NULL) {
		::freeaddrinfo(addr);
	}

	if (sock != -1) {
		int flag = 1;
#if TCP_NODELAY
		rc = ::setsockopt(sock,IPPROTO_TCP,TCP_NODELAY,&flag, sizeof(flag));
		if (rc == -1) {
			rc = errno;
			LOG4CXX_WARN(logger,"Open port " << getPortName()
					<< ": setsockopt TCP_NODELAY error:" << strerror(rc));
		}
#endif // #if TCP_NODELAY
#if TCP_QUICKACK
		flag = 1;
		rc = ::setsockopt(sock,IPPROTO_TCP,TCP_QUICKACK,&flag, sizeof(flag));
		if (rc == -1) {
			rc = errno;
			LOG4CXX_WARN(logger,"Open port " << getPortName()
					<< ": setsockopt TCP_QUICKACK error:" << strerror(rc));
		}
#endif // #if TCP_QUICKACK

	}

	{
		DeviceHandleAccess devAcc(*this);
		devAcc.deviceHandle = sock;
	}

	LOG4CXX_INFO(logger,"Open port " << getPortName() << ": Connected to host");


}

} /* namespace io */
} /* namespace openEV */
