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
#include <cstring>
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

#include "fmt/format.h"

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

std::string currPropertyName;

	try {

		currPropertyName = hostPropertyName;
		auto prop = portConfiguration.searchProperty(currPropertyName);
		tcpAddr = prop->getStringValue();

		LOG4CXX_DEBUG(logger,"Configure port " << getPortName()
				<< ": host = \"" << tcpAddr << '\"');

		currPropertyName = portPropertyName;
		prop = portConfiguration.searchProperty(currPropertyName);
		tcpPort = prop->getStringValue();

		LOG4CXX_DEBUG(logger,"Configure port " << getPortName()
				<< ": port = \"" << tcpPort << '\"');
	} catch (Properties4CXX::ExceptionPropertyNotFound const &e) {
		auto str = fmt::format(_(
				"Error configuring port \"(0)\" of type {1}: Mandatory property {3} is missing."),
				getPortName(),getPortType(),currPropertyName);
	}

}

/** \brief Manages the dynamically allocated memory from getaddrinfo
 *
 * Use an object of this struct to release the dynamically allocated memory
 * in all circumstances, including the numerous cases when an exception is thrown.
 */
struct AddrInfoManagerTCP final {
	struct addrinfo addrHint;
	struct addrinfo *addr = nullptr;

	AddrInfoManagerTCP () {
		::memset (&addrHint,0,sizeof(addrHint));
	}

	~AddrInfoManagerTCP () {
		if (addr != nullptr) {
			::freeaddrinfo(addr);
		}
	}
};

void TCPPort::openInternal() {

	int sock = -1;
	int rc;
	AddrInfoManagerTCP addrMgr;

	// Allow TCP as well as TCP V6; Do not insist on TCP V4
	// addrMgr.addrHint.ai_family = AF_INET;

	// But insist on TCP; do not allow UDP
	addrMgr.addrHint.ai_socktype = SOCK_STREAM;

	rc = ::getaddrinfo(tcpAddr.c_str(),tcpPort.c_str(),&addrMgr.addrHint,&addrMgr.addr);

	if (rc != 0 || addrMgr.addr == nullptr) {
		std::string str;

		if (rc != 0) {
			str = fmt::format(_(
				"{0}: Error resolving host \"{1}\" and/or TCP port \"{2}\" for I/O port \"{3}\". Error code = {4}: {5}"),
				__PRETTY_FUNCTION__, tcpAddr, tcpPort, getPortName(), rc, gai_strerror(rc));
		} else {
			str = fmt::format(_(
				"{0}: Error resolving host \"{1}\" and/or TCP port \"{2}\" for I/O port \"{3}\": "
				"getaddrinfo() returned 0 but *_pai is NULL."
					),
				__PRETTY_FUNCTION__, tcpAddr, tcpPort, getPortName());
		}

		LOG4CXX_ERROR(logger,str);
		throw GliderVarioPortOpenException (
				__FILE__,
				__LINE__,
				str.c_str(),
				rc);
	}

	auto sockType = addrMgr.addr->ai_socktype;

	if (!isBlocking()) {
		sockType |= SOCK_NONBLOCK;
	}

	sock = ::socket(addrMgr.addr->ai_family,sockType,addrMgr.addr->ai_protocol);

	if (sock == -1) {
		rc = errno;
		auto str = fmt::format(_(
				"{0}: Error creating a {1} socket for port \"{2}\". errno = {3}: {4}"),
				__PRETTY_FUNCTION__,"TCP", getPortName(), rc, strerror(rc));
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioPortOpenException (
				__FILE__,
				__LINE__,
				str.c_str(),
				rc);
	}

	rc = ::connect(sock,addrMgr.addr->ai_addr,addrMgr.addr->ai_addrlen);
	if (rc == -1) {
		rc = errno;
		auto str = fmt::format(_(
				"{0}: Error connect TCP socket to host \"{1}\" on TCP port \"{2}\" for I/O port \"{3}\". errno = {4}: {5}"),
				__PRETTY_FUNCTION__, tcpAddr, tcpPort, getPortName(), rc, strerror(rc));
		LOG4CXX_ERROR(logger,str);
		::close (sock);
		sock = -1;
		throw GliderVarioPortOpenException (
				__FILE__,
				__LINE__,
				str.c_str(),
				rc);
	}


	{
		DeviceHandleAccess devAcc(*this);
		devAcc.deviceHandle = sock;
	}

	LOG4CXX_INFO(logger,fmt::format(_("Port \"{0}\": Connected to host {1} on TCP port {2}.")));

	int flag = 1;
#if TCP_NODELAY
	rc = ::setsockopt(sock,IPPROTO_TCP,TCP_NODELAY,&flag, sizeof(flag));
	if (rc == -1) {
		rc = errno;
		LOG4CXX_WARN(logger,fmt::format(_("Open I/O port \"{0}\": setsockopt {1} error: errno = {2} : {3}"),
				getPortName(),"TCP_NODELAY",rc,strerror(rc)));
	}
#endif // #if TCP_NODELAY
#if TCP_QUICKACK
	flag = 1;
	rc = ::setsockopt(sock,IPPROTO_TCP,TCP_QUICKACK,&flag, sizeof(flag));
	if (rc == -1) {
		rc = errno;
		LOG4CXX_WARN(logger,fmt::format(_("Open I/O port \"{0}\": setsockopt {1} error: errno = {2} : {3}"),
				getPortName(),"TCP_QUICKACK",rc,strerror(rc)));
	}
#endif // #if TCP_QUICKACK

}

} /* namespace io */
} /* namespace openEV */
