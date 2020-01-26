/*
 * TCPPort.cpp
 *
 *  Created on: Dec 23, 2019
 *      Author: hor
 */

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
