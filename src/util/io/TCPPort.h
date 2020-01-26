/*
 * TCPPort.h
 *
 *  Created on: Dec 23, 2019
 *      Author: hor
 */

#ifndef UTIL_IO_TCPPORT_H_
#define UTIL_IO_TCPPORT_H_

#include "util/io/StreamPort.h"

namespace openEV {
namespace io {

class TCPPort: public StreamPort {
public:

	static constexpr char const* TcpPortType = "TCP-Port";

	TCPPort(char const* portName);
	virtual ~TCPPort();

	virtual void open() override;
	virtual void configurePort(
			const Properties4CXX::Properties &globalConfiguration,
			const Properties4CXX::Properties &portConfiguration) override;

	static PortBase* tcpPortConstructor(
			char const* portName,
			Properties4CXX::Properties const &portProp);
	static void registerTcpPortType();

private:

protected:
	std::string tcpAddr;
	std::string tcpPort;
};

} /* namespace io */
} /* namespace openEV */

#endif /* UTIL_IO_TCPPORT_H_ */
