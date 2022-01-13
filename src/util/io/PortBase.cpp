/*
 * PortBase.cpp
 *
 *  Created on: Jun 20, 2019
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

#include <sstream>
#include <cstring>

#include <OEVCommon.h>
#include "util/io/PortBase.h"


#if defined HAVE_LOG4CXX_H
static log4cxx::LoggerPtr logger = 0;

static inline void initLogger() {
	if (!logger) {
		logger = log4cxx::Logger::getLogger("openEV.IO.PortBase");
	}
}

#endif


namespace openEV {
namespace io {

static std::string const DevicePropertyName = "device";
static std::string const BlockingPropertyName = "blocking";

#if !defined DOXYGEN
PortBase::StatusEnumHelperClass PortBase::StatusEnumHelperObj;
#endif
PortBase::PortTypeMap PortBase::typeMap;
PortBase::PortMap PortBase::portMap;

PortBase::PortBase(
		char const* portName,
		char const* portType
		) :
	portName{portName},
	deviceName{portName}, // set deviceName by default to portName.
	portType{portType}
{
#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	LOG4CXX_DEBUG(logger,__PRETTY_FUNCTION__
			<< "(portName=" << portName
			<< ", portType=" << portType << ')');

}

PortBase::~PortBase() {

}

void PortBase::addPortType (const char* portType, PortConstructor portConstruct) {

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	LOG4CXX_INFO(logger,__PRETTY_FUNCTION__
			<< "(portType=" << portType);

	auto rc = typeMap.emplace(portType,portConstruct);
	if (!rc.second) {
		std::ostringstream os;
		os << "Port type '" << portType << "' is defined more than once";
		LOG4CXX_ERROR(logger,os.str());
		throw GliderVarioPortException (__FILE__,__LINE__,os.str().c_str());
	}
}

void PortBase::addPort (PortBasePtr port) {

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	LOG4CXX_INFO(logger,__PRETTY_FUNCTION__
			<< "(portName=" << port->portName
			<< ", portType=" << port->portType << ')');

	auto rc = portMap.emplace(port->portName,port);
	if (!rc.second) {
		std::ostringstream os;
		os << "Port '" << port->portName << "' is defined more than once";
		LOG4CXX_ERROR(logger,os.str());
		throw GliderVarioPortConfigException (__FILE__,__LINE__,os.str().c_str());
	}
}

void PortBase::loadSinglePort (
		Properties4CXX::Properties const &globalProperties,
		Properties4CXX::Property const &portProperty,
		std::string const &portName) {

	PortBase* newPort = nullptr;
	LOG4CXX_INFO(logger,"Load port '" << portProperty.getPropertyName() << '\'');

	if (portProperty.isStruct()) {
		Properties4CXX::Properties const &portStruct = portProperty.getPropertiesStructure();
		try {
			Properties4CXX::Property const *typeProp = portStruct.searchProperty("type");

			LOG4CXX_DEBUG(logger, "port type = " << typeProp->getStringValue());

			// look up the port type
			auto portTypeIter = typeMap.find(typeProp->getStringValue());
			if (portTypeIter == typeMap.end()) {
				LOG4CXX_ERROR(logger,"Type '" << typeProp->getStringValue() << "' of port '" << portProperty.getPropertyName()
						<< "' does not exist.");
				LOG4CXX_ERROR(logger,"  Port '" << portProperty.getPropertyName() << "' will not be created.");
			} else { // if (portTypeIter == typeMap.end())
				PortConstructor portConstructor = portTypeIter->second;
				newPort = portConstructor(portName.c_str(),portStruct);
			} // if (portTypeIter == typeMap.end())

		} catch (Properties4CXX::ExceptionPropertyNotFound const &) {
			LOG4CXX_ERROR(logger,"Port '" << portProperty.getPropertyName() << "' has no 'type' property.");
			LOG4CXX_ERROR(logger,"  Port '" << portProperty.getPropertyName() << "' will not be created.");
		}
	} else { // if (property.isStruct())
		LOG4CXX_ERROR(logger,"Port configuration '" << portProperty.getPropertyName() << "' is not a structure.");
		LOG4CXX_ERROR(logger,"  Port '" << portProperty.getPropertyName() << "' will not be created.");
	} //if (property.isStruct())

	if (newPort) {
		// The port was created.
		PortBasePtr portPtr (newPort);
		Properties4CXX::Properties const &portStruct = portProperty.getPropertiesStructure();

		// Configure the port
		// If an exception is thrown here, no worries.
		// The shared pointer object portPtr will take care of deleting the dangling port object.
		portPtr->configurePort(globalProperties,portStruct);

		// Add it to the list of ports.
		addPort (portPtr);

		// Load default properties
		newPort->deviceName = portStruct.getPropertyValue(DevicePropertyName,newPort->getPortName().c_str());
		newPort->blocking = portStruct.getPropertyValue(BlockingPropertyName,true);

		LOG4CXX_DEBUG(logger, "deviceName = " << newPort->deviceName);
		LOG4CXX_DEBUG(logger, "blocking = " << newPort->blocking);

	}
}

void PortBase::loadPorts(Properties4CXX::Properties const &properties) {

#if defined HAVE_LOG4CXX_H
	initLogger();
#endif /* HAVE_LOG4CXX_H */

	try {
		auto ioPorts = properties.searchProperty("IOPorts");
		if (ioPorts->isStruct()) {
			Properties4CXX::Properties const &portList = ioPorts->getPropertiesStructure();
			auto portPropIter = portList.getFirstProperty();
			while (portPropIter != portList.getListEnd()) {
				try {
					Properties4CXX::Property const &portProp = portList.getPropertyFromIterator(portPropIter);
					auto portName = portPropIter->first;
					loadSinglePort(properties,portProp,portName);
				} catch (GliderVarioPortConfigException const& e) {
					LOG4CXX_ERROR (logger, "Exception when loading port "
							<< portPropIter->second->getPropertyName()
							<< " in source " << e.getSource() << " at line " << e.getLine()
							<< ": " << e.getDescription());
				}
				portPropIter++;
			}
		} else {
			LOG4CXX_ERROR(logger,"Property 'IOPorts' is not a structure. No I/O ports will be created.");
		}
	} catch (Properties4CXX::ExceptionPropertyNotFound const& e) {
		LOG4CXX_WARN(logger,"Properties section 'IOPorts' does not exist. No I/O ports will be created.");
	}

}

PortBase* PortBase::getPortByName(std::string const & portName) {

	auto rc = portMap.find(portName);

	if (rc == portMap.end()) {
		std::ostringstream str;
		str << "Cannot find IO port \"" << portName << "\"";
		LOG4CXX_ERROR (logger,str.str());
		throw GliderVarioPortConfigException(__FILE__,__LINE__,str.str().c_str());
	}

	return rc->second.get();

}

void PortBase::open() {
	devHandleMutex.lock();

	if (status == CLOSED) {

		try {
		openInternal();
		} catch (std::exception const &c) {
			devHandleMutex.unlock();
			throw;
		}
		status = OPEN;
	}

	LOG4CXX_INFO(logger,"Port" << portName << ':' << portType << ": Opened port device \"" << deviceName << "\"");

	devHandleMutex.unlock();
}

void PortBase::openInternal() {

	deviceHandle = ::open(getDeviceName().c_str(),deviceOpenFlags);

	if (deviceHandle == -1) {
		int err = errno;
		std::ostringstream str;

		str << "Port" << portName << ':' << portType << ": Cannot open device \"" << deviceName << "\"";
		LOG4CXX_ERROR(logger,"Port" << portName << ':' << portType << ": Opening \"" << deviceName << "\" failed. errno = " << err << ": " << strerror(err));

		if (err == ENOENT) {
			status = ERR_NO_DEVICE;
			throw GliderVarioPortDontExistException (__FILE__,__LINE__,str.str().c_str(),err);
		} else { // if (err == ENOENT)
			status = ERR_IO_TEMP;
			throw GliderVarioPortDontExistException (__FILE__,__LINE__,str.str().c_str(),err);
		}
	} else {
		LOG4CXX_DEBUG(logger,"Port " << portName << ":" << deviceName << " opened. Handle = " << deviceHandle);
	}

}

void PortBase::close() noexcept {
	devHandleMutex.lock();

	closeInternal();
	status = CLOSED;

	LOG4CXX_INFO(logger,"Port" << portName << ':' << portType << ": Closed port device \"" << portName << "\"");

	devHandleMutex.unlock();
}

void PortBase::closeInternal() noexcept {
	if (deviceHandle > 0) {
		::close(deviceHandle);
	}
	deviceHandle = 0;
}

void PortBase::recoverError() {
	close();
	open();
}



} /* namespace io */
} /* namespace openEV */
