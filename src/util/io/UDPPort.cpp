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

	if (peerAd != nullptr) {
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
		// An undefined local address means I am listening on all network adapters.
		localAddr = "0.0.0.0";
	}

	try {
		auto prop = portConfiguration.searchProperty(localPortPropertyName);
			localPort = prop->getStringValue();
			localPortDefined = true;
			LOG4CXX_DEBUG(logger,"Configure port " << getPortName()
					<< ": peerAddr = \"" << peerAddr << '\"');
	} catch (...) {}

	// Some sanity checks upfront to avoid later confusion and strange error messages.
	if ((peerPortDefined && !peerAddrDefined) || (!peerPortDefined && peerAddrDefined)) {
		auto str = fmt::format(_("Configuration error for port {0}: Both peer host and peer port must be defined or none."));
		LOG4CXX_ERROR (logger,str);
		throw GliderVarioPortConfigException (__FILE__,__LINE__, str.c_str());
	}

	if (!peerPortDefined && !localPortDefined ){
		auto str = fmt::format(_("{0}: I/O port {1} error: neither a local nor a remote port and/or host address were defined."),
				__PRETTY_FUNCTION__,getPortName());
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioPortConfigException (
				__FILE__,
				__LINE__,
				str.c_str());
	}

}

/** \brief Manages the dynamically allocated memory from getaddrinfo
 *
 * Use an object of this struct to release the dynamically allocated memory
 * in all circumstances, including the numerous cases when an exception is thrown.
 */
struct AddrInfoManagerUDP final {
	struct addrinfo addrHint;
	struct addrinfo *localAd = nullptr;

	AddrInfoManagerUDP () {
		::memset (&addrHint,0,sizeof(addrHint));
	}

	~AddrInfoManagerUDP () {
		if (localAd != nullptr) {
			::freeaddrinfo(localAd);
		}
	}
};

void UDPPort::openInternal() {

	int sock = -1;
	int rc = 0;
	AddrInfoManagerUDP addrMgr;

	// Allow IP V4 as well as IP V6; Do not insist on IP V4
	// addrMgr.addrHint.ai_family = AF_INET;

	// But insist on UDP; do not allow TCP
	addrMgr.addrHint.ai_socktype = SOCK_DGRAM;

	// If the local address is undefined it defaults to the wildcard address "0.0.0.0"
	if (localPortDefined) {

		rc = ::getaddrinfo(localAddr.c_str(),localPort.c_str(),&addrMgr.addrHint,&addrMgr.localAd);

		if (rc != 0) {
			auto str = fmt::format(_(
					"{0}: Port {1}: getaddrinfo() error for local host {2}, local UDP port {3} = {4} : {5}"),
					__PRETTY_FUNCTION__,getPortName(),localAddr,localPort,rc,gai_strerror(rc));
			LOG4CXX_ERROR(logger,str);
			throw GliderVarioPortOpenException (__FILE__, __LINE__, str.c_str());
		}

		auto sockType = addrMgr.localAd->ai_socktype;

		if (!isBlocking()) {
			sockType |= SOCK_NONBLOCK;
		}

		sock = ::socket(addrMgr.localAd->ai_family,sockType,addrMgr.localAd->ai_protocol);

		if (sock == -1) {
			rc = errno;
			auto str = fmt::format(_("{0}: Error creating a {1} socket for port \"{2}\". errno = {3}: {4}"),
					__PRETTY_FUNCTION__,"UDP",getPortName(),rc,strerror(rc));
			LOG4CXX_ERROR(logger,str);
			throw GliderVarioPortOpenException (
					__FILE__,
					__LINE__,
					str.c_str(),
					rc);
		}

		rc = ::bind(sock,addrMgr.localAd->ai_addr,addrMgr.localAd->ai_addrlen);
		if (rc == -1) {
			rc = errno;
			auto str = fmt::format(_(
					"{0}: Port {1}: bind() error for local host {2}, UDP port {3} = {4} : {5}"),
					__PRETTY_FUNCTION__,getPortName(),localAddr,localPort,rc,strerror(rc));
			::close (sock);
			sock = -1;
			LOG4CXX_ERROR(logger,str);
			throw GliderVarioPortOpenException (__FILE__, __LINE__, str.c_str(),rc);
		} else {
			socketBound = true;
		}


	} // if (localPortDefined)

	// For the peer address I need the IP address and the port.
	if (peerPortDefined && peerAddrDefined) {

		rc = ::getaddrinfo(peerAddr.c_str(),peerPort.c_str(),&addrMgr.addrHint,&peerAd);

		if (rc != 0) {
			auto str = fmt::format(_(
					"{0}: Port {1}: getaddrinfo() error for peer host {2}, UDP port {3} = {4} : {5}"),
					__PRETTY_FUNCTION__,getPortName(),peerAddr,peerPort,rc,gai_strerror(rc));
			LOG4CXX_ERROR(logger,str);
			throw GliderVarioPortOpenException (__FILE__, __LINE__, str.c_str());
		}

		// If the socket has not yet been created do it now.
		if (sock == -1) {
			auto sockType = peerAd->ai_socktype;

			if (!isBlocking()) {
				sockType |= SOCK_NONBLOCK;
			}

			sock = ::socket(peerAd->ai_family,sockType,peerAd->ai_protocol);

			if (sock == -1) {
				rc = errno;
				auto str = fmt::format(_("{0}: Error creating a {1} socket for port \"{2}\". errno = {3}: {4}"),
						__PRETTY_FUNCTION__,"UDP",getPortName(),rc,strerror(rc));
				LOG4CXX_ERROR(logger,str);
				throw GliderVarioPortOpenException (
						__FILE__,
						__LINE__,
						str.c_str(),
						rc);
			}
		} // if (sock == -1)

		socketConnected = true;


	} // if (peerPortDefined && peerAddrDefined)

	if (sock != -1) {
		DeviceHandleAccess devAcc(*this);
		devAcc.deviceHandle = sock;
	} else {
		auto str = fmt::format(_("{0}: I/O port {1} error: neither a local nor a remote port and/or host address were defined."),
				__PRETTY_FUNCTION__,getPortName());
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioPortOpenException (
				__FILE__,
				__LINE__,
				str.c_str());
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

	if (!peerPortDefined) {
		auto str = fmt::format(_("{0}: For I/O port {1} no peer port and/or address are defined. "
				"I do not know where to send data to. Closing the Port."),
				__PRETTY_FUNCTION__,getPortName());
		close();
		LOG4CXX_ERROR(logger,str);
		throw GliderVarioPortWriteException(__FILE__,__LINE__,str.c_str());
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
				auto str = fmt::format(_("Port \"{0}\" of type \"{1}\": {4} error {2}: {3}"),
						getPortName(), getPortType(), err, strerror(err),"sendto()");
				LOG4CXX_ERROR (logger,str);
				throw GliderVarioPortWriteException(__FILE__,__LINE__,str.c_str(),err);
			}
		}

	} while (err == EINTR);

	LOG4CXX_DEBUG(logger,"UDPPort::send: ret = " << ret);

	return ret;
}

} /* namespace io */
} /* namespace openEV */
