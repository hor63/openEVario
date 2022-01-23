/*
 * UDPPort.cpp
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

#include "util/io/UDPPort.h"

#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.IO.UDPPort");
	}
}

#endif


namespace openEV {
namespace io {

static std::string const peerAddressPropertyName = "peerAddress";
static std::string const localAddressPropertyName = "localAddress";
static std::string const peerPortPropertyName = "peerPort";
static std::string const localPortPropertyName = "localPort";

UDPPort::UDPPort(char const* portName)
	: DatagramPort(portName,UdpPortType)
{
#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	LOG4CXX_DEBUG(logger,__PRETTY_FUNCTION__
			<< "(portName=" << getPortName()
			<< ", portType=" << getPortType() << ')');

}

UDPPort::~UDPPort() {

	if (peerAd != NULL) {
		::freeaddrinfo(peerAd);
	}

}

PortBase* UDPPort::udpPortConstructor(
		char const* portName,
		Properties4CXX::Properties const &portProp) {

	return new UDPPort(portName);
}

void UDPPort::registerUdpPortType() {
	addPortType(UdpPortType,udpPortConstructor);
}

void UDPPort::configurePort(
		const Properties4CXX::Properties &globalConfiguration,
		const Properties4CXX::Properties &portConfiguration) {


	try {
		auto prop = portConfiguration.searchProperty(peerAddressPropertyName);
			peerAddr = prop->getStringValue();
			peerAddrDefined = true;
			LOG4CXX_DEBUG(logger,"Configure port " << getPortName()
					<< ": peerAddr = \"" << peerAddr << '\"');
	} catch (...) {}

	try {
		auto prop = portConfiguration.searchProperty(peerPortPropertyName);
			peerPort = prop->getStringValue();
			peerPortDefined = true;
			LOG4CXX_DEBUG(logger,"Configure port " << getPortName()
					<< ": peerAddr = \"" << peerAddr << '\"');
	} catch (...) {}

	try {
		auto prop = portConfiguration.searchProperty(localAddressPropertyName);
			localAddr = prop->getStringValue();
			localAddrDefined = true;
			LOG4CXX_DEBUG(logger,"Configure port " << getPortName()
					<< ": peerAddr = \"" << peerAddr << '\"');
	} catch (...) {
		// An undefined
		localAddr = "0.0.0.0";
	}

	try {
		auto prop = portConfiguration.searchProperty(localPortPropertyName);
			localPort = prop->getStringValue();
			localPortDefined = true;
			LOG4CXX_DEBUG(logger,"Configure port " << getPortName()
					<< ": peerAddr = \"" << peerAddr << '\"');
	} catch (...) {}


}

void UDPPort::openInternal() {

	int sock = -1;
	int rc = 0;
	struct addrinfo addrHint;

	::memset (&addrHint,0,sizeof(addrHint));

	// Allow IP V4 as well as IP V6; Do not insist on IP V4
	// addrHint.ai_family = AF_INET;

	// But insist on UDP; do not allow TCP
	addrHint.ai_socktype = SOCK_DGRAM;

	// If the local address is undefined it defaults to the wildcard address "0.0.0.0"
	if (localPortDefined) {
		struct addrinfo *localAd = nullptr;

		rc = ::getaddrinfo(localAddr.c_str(),localPort.c_str(),&addrHint,&localAd);

		if (rc == 0) {
			auto sockType = localAd->ai_socktype;

			if (!isBlocking()) {
				sockType |= SOCK_NONBLOCK;
			}

			sock = ::socket(localAd->ai_family,sockType,localAd->ai_protocol);

			if (sock == -1) {
				rc = errno;
				::freeaddrinfo(localAd);
				LOG4CXX_ERROR(logger,"Open port " << getPortName()
						<< ": socket() error: " << rc << '=' << strerror(rc));
				throw GliderVarioPortOpenException (
						__FILE__,
						__LINE__,
						"Error in socket()",
						rc);
			}

			rc = ::bind(sock,localAd->ai_addr,localAd->ai_addrlen);
			if (rc == -1) {
				rc = errno;
				LOG4CXX_ERROR(logger,"Open port " << getPortName()
						<< ": bind() local address error: " << rc << '=' << strerror(rc));
				::close (sock);
				sock = -1;
			} else {
				socketBound = true;
			}

		} else { // if (rc == 0)
			std::ostringstream ostr;
			ostr << "getaddrinfo() local address error: " << gai_strerror(rc);
			LOG4CXX_ERROR(logger,"Open port " << getPortName()
					<< ostr.str());
		} // if (rc == 0)

		if (localAd != NULL) {
			::freeaddrinfo(localAd);
		}
	} // if (localPortDefined)

	// For the peer address I need the IP address and the port.
	if (peerPortDefined && peerAddrDefined) {

		rc = ::getaddrinfo(peerAddr.c_str(),peerPort.c_str(),&addrHint,&peerAd);

		if (rc == 0) {

			// If the socket has not yet been created do it now.
			if (sock == -1) {
				auto sockType = peerAd->ai_socktype;

				if (!isBlocking()) {
					sockType |= SOCK_NONBLOCK;
				}

				sock = ::socket(peerAd->ai_family,sockType,peerAd->ai_protocol);

				if (sock == -1) {
					rc = errno;
					::freeaddrinfo(peerAd);
					peerAd = nullptr;
					LOG4CXX_ERROR(logger,"Open port " << getPortName()
							<< ": socket() error: " << rc << '=' << strerror(rc));
					throw GliderVarioPortOpenException (
							__FILE__,
							__LINE__,
							"Error in socket()",
							rc);
				}
			} // if (sock == -1)

			socketConnected = true;

		} else { // if (rc == 0)
			std::ostringstream ostr;
			ostr << "getaddrinfo() peer address error: " << gai_strerror(rc);
			LOG4CXX_ERROR(logger,"Open port " << getPortName()
					<< ostr.str());
		} // if (rc == 0)

	} // if (peerPortDefined && peerAddrDefined)

	if (sock != -1) {
		DeviceHandleAccess devAcc(*this);
		devAcc.deviceHandle = sock;
	} else {
		throw GliderVarioPortOpenException (
				__FILE__,
				__LINE__,
				"Error: Could neither bind local address nor connect destination address to socket",
				rc);

	}

	LOG4CXX_INFO(logger,"Open port " << getPortName() << " successful");


}

ssize_t UDPPort::send(uint8_t *buffer, size_t bufLen) {
	ssize_t ret;
	int err;
	DeviceHandleAccess devHandleAccess (*this);
	int flags = 0;

	if (!isBlocking()) {
		flags |= MSG_DONTWAIT;
	}

	do {
		err = 0;

		ret = ::sendto(devHandleAccess.deviceHandle,buffer,bufLen,flags,peerAd->ai_addr,peerAd->ai_addrlen);

		if (ret == -1) {
			err = errno;

			switch (err) {

			case EINTR:
				LOG4CXX_DEBUG (logger,"Port" << getPortName() << ':' << getPortType() << ": sendto interrupted with EINTR. Repeat ::send() ");
				break;

			case EWOULDBLOCK:
#if EWOULDBLOCK != EAGAIN
			case EAGAIN:
#endif
				ret = 0;
				break;
			default:
				std::ostringstream str;

				str << "Port" << getPortName() << ':' << getPortType() << ": sendto error " << err << ":" << strerror(err);
				LOG4CXX_ERROR (logger,str.str());
				throw GliderVarioPortWriteException(__FILE__,__LINE__,str.str().c_str(),err);
			}
		}

	} while (err == EINTR);

	LOG4CXX_DEBUG(logger,"UDPPort::send: ret = " << ret);

	return ret;
}

} /* namespace io */
} /* namespace openEV */
